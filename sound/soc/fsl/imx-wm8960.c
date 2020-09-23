/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/syscon.h>
#include "../codecs/wm8960.h"
//#include "fsl_sai.h"

#include "imx-audmux.h"

#define DAI_NAME_SIZE	32

struct imx_wm8960_data {
	struct snd_soc_dai_link dai[3];
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct clk *codec_clk;
	unsigned int clk_frequency;
	bool is_codec_master;
	bool is_stream_in_use[2];
	struct regmap *gpr;
	unsigned int hp_det[2];
};

struct imx_priv {
	int hp_set_gpio;
	int hp_active_low;
	struct snd_kcontrol *headset_kctl;
	struct snd_soc_component *component;
	struct platform_device *pdev;
	struct platform_device *asrc_pdev;
	struct snd_card *snd_card;
	struct snd_pcm_substream *first_stream;
	struct snd_pcm_substream *second_stream;
	u32 asrc_rate;
	u32 asrc_format;
};

static struct imx_priv card_priv;

static struct snd_soc_jack imx_hp_set;
static struct snd_soc_jack_pin imx_hp_set_pins[] = {
	{
		.pin = "Headset Jack",
		.mask = SND_JACK_HEADSET,
	},
};
static struct snd_soc_jack_gpio imx_hp_set_gpio = {
	.name = "headset detect",
	.report = SND_JACK_HEADSET,
	.debounce_time = 250,
	.invert = 1,
};

static int hp_set_status_check(void *data)
{
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	char *envp[3], *buf;
	int hp_status, ret;

	if (!gpio_is_valid(priv->hp_set_gpio))
		return 0;

	hp_status = 0;// gpio_get_value(priv->hp_set_gpio) ? 1 : 0;
	buf = kmalloc(32, GFP_ATOMIC);
	if (!buf) {
		dev_err(&pdev->dev, "%s kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	if (hp_status != priv->hp_active_low) {
		snprintf(buf, 32, "STATE=%d", 2);
		snd_soc_dapm_disable_pin(snd_soc_component_get_dapm(priv->component), "Ext Spk");
		snd_soc_dapm_disable_pin(snd_soc_component_get_dapm(priv->component), "Main MIC");
		ret = imx_hp_set_gpio.report;

		/*
		 *  As the hp MIC only connect the input for left channel, we
		 *  need to route it for right channel.
		 */
		snd_soc_component_update_bits(priv->component, WM8960_ADDCTL1, 3<<2, 1<<2);

		snd_kctl_jack_report(priv->snd_card, priv->headset_kctl, 1);
	} else {
		snprintf(buf, 32, "STATE=%d", 0);
		snd_soc_dapm_enable_pin(snd_soc_component_get_dapm(priv->component), "Ext Spk");
		snd_soc_dapm_enable_pin(snd_soc_component_get_dapm(priv->component), "Main MIC");
		ret = 0;
		printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		/*
		 *  As the Main MIC only connect the input for right channel,
		 *  we need to route it for left channel.
		 */
		snd_soc_component_update_bits(priv->component, WM8960_ADDCTL1, 3<<2, 2<<2);

		snd_kctl_jack_report(priv->snd_card, priv->headset_kctl, 0);
	}

	envp[0] = "NAME=headset";
	envp[1] = buf;
	envp[2] = NULL;
	kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
	kfree(buf);

	return ret;
}

static const struct snd_soc_dapm_widget imx_wm8960_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headset Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Hp MIC", NULL),
	SND_SOC_DAPM_MIC("Main MIC", NULL),
};

static int imx_wm8960_gpio_init(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd = list_first_entry(
				&card->rtd_list, struct snd_soc_pcm_runtime, list);
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
//	struct snd_soc_codec *codec = codec_dai->codec;
	struct imx_priv *priv = &card_priv;
//	int ret;
	priv->component = codec_dai->component;
	//printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
	/*if (gpio_is_valid(priv->hp_set_gpio)) {
		imx_hp_set_gpio.gpio = priv->hp_set_gpio;
		imx_hp_set_gpio.jack_status_check = hp_set_status_check;

		ret = snd_soc_card_jack_new(card, "Headphone Jack",
				SND_JACK_HEADPHONE, &imx_hp_set,
				imx_hp_set_pins, ARRAY_SIZE(imx_hp_set_pins));
		if (ret)
			return ret;
		ret = snd_soc_jack_add_pins(&imx_hp_set,
			ARRAY_SIZE(imx_hp_set_pins), imx_hp_set_pins);
		if (ret)
			return ret;
		ret = snd_soc_jack_add_gpios(&imx_hp_set, 1,
						&imx_hp_set_gpio);
		if (ret)
			return ret;
	}*/
	//printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
	return 0;
}

static ssize_t headphone_show(struct device_driver *dev, char *buf)
{
	struct imx_priv *priv = &card_priv;
	int hp_status;

	if (!gpio_is_valid(priv->hp_set_gpio)) {
		strcpy(buf, "no detect gpio connected\n");
		printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		return strlen(buf);
	}

	/* Check if headphone is plugged in */
	hp_status = 0;// gpio_get_value(priv->hp_set_gpio) ? 1 : 0;

	if (hp_status != priv->hp_active_low)
		strcpy(buf, "Headphone\n");
	else
		strcpy(buf, "Speaker\n");
	printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
	return strlen(buf);
}

static ssize_t micphone_show(struct device_driver *dev, char *buf)
{
	struct imx_priv *priv = &card_priv;
	int hp_status;

	if (!gpio_is_valid(priv->hp_set_gpio)) {
		strcpy(buf, "no detect gpio connected\n");
		return strlen(buf);
	}

	/* Check if micphone is plugged in */
	hp_status = 0;//gpio_get_value(priv->hp_set_gpio) ? 1 : 0;

	if (hp_status != priv->hp_active_low)
		strcpy(buf, "Hp MIC\n");
	else
		strcpy(buf, "Main MIC\n");

	return strlen(buf);
}
//static DRIVER_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);
//static DRIVER_ATTR(micphone, S_IRUGO | S_IWUSR, show_micphone, NULL);
static DRIVER_ATTR_RO(headphone);
static DRIVER_ATTR_RO(micphone);

#if 0
static void wm8960_init(struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_pcm_runtime *rtd = snd_soc_get_pcm_runtime(card, card->dai_link[0].name);;
//	struct imx_priv *priv = &card_priv;
//	struct device *dev = &priv->pdev->dev;
	struct snd_soc_card *card =rtd->card;//platform_get_drvdata(priv->pdev);
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);


	/*
	 * codec ADCLRC pin configured as GPIO, DACLRC pin is used as a frame
	 * clock for ADCs and DACs
	 */
	snd_soc_component_update_bits(priv->component, WM8960_IFACE2, 1<<6, 1<<6);

	/*
	 * GPIO1 used as headphone detect output
	 */
	snd_soc_component_update_bits(priv->component, WM8960_ADDCTL4, 7<<4, 3<<4);

	/*
	 * Enable headphone jack detect
	 */
	snd_soc_component_update_bits(priv->component, WM8960_ADDCTL2, 1<<6, 1<<6);
	snd_soc_component_update_bits(priv->component, WM8960_ADDCTL2, 1<<5, data->hp_det[1]<<5);
	snd_soc_component_update_bits(priv->component, WM8960_ADDCTL4, 3<<2, data->hp_det[0]<<2);
	snd_soc_component_update_bits(codec, WM8960_ADDCTL1, 3, 3);

	/*
	 * route left channel to right channel in default.
	 */
	snd_soc_update_bits(priv->component, WM8960_ADDCTL1, 3<<2, 1<<2);
}
#endif

/* -1 for reserved value */
static const int sysclk_divs[] = { 1, -1, 2, -1 };

/* Multiply 256 for internal 256 div */
static const int dac_divs[] = { 256, 384, 512, 768, 1024, 1408, 1536 };

/* Multiply 10 to eliminate decimials */
static const int bclk_divs[] = {
	10, 15, 20, 30, 40, 55, 60, 80, 110,
	120, 160, 220, 240, 320, 320, 320
};

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct imx_priv *priv = &card_priv;
	struct device *dev = &priv->pdev->dev;
	struct snd_soc_card *card = platform_get_drvdata(priv->pdev);
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int sample_rate = params_rate(params);

	unsigned int fmt,sysclk, pll_out;
	int ret = 0;



//	if (params_channels(params) == 1)
//		bclk *= 2;

/*	data->is_stream_in_use[tx] = true;

	if (data->is_stream_in_use[!tx])

		return 0;
*/
	if (!priv->first_stream) {
			priv->first_stream = substream;
			//printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		} else {
			priv->second_stream = substream;
			//printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
			/* We suppose the two substream are using same params */
			return 0;
		}

	if (data->is_codec_master)
	{
		fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
	}
	else
	{
		fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;
	}

	/* set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
		if (ret) {
			dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
			return ret;
		}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
		//printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		return ret;
	}

	if (!data->is_codec_master) {
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 2, params_width(params));
		if (ret) {
			dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
//			printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
			return ret;
		}

		ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_OUT);
		if (ret) {
			dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
 		    printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
			return ret;
		}
		return 0;
								}
	else {
		ret = snd_soc_dai_set_sysclk(codec_dai, 0, 0, SND_SOC_CLOCK_IN);
		if (ret) {
			dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
			printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
			return ret;
		}
	      	  	  	  	  	  	}

	data->clk_frequency = clk_get_rate(data->codec_clk);
	//printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
	/* Use MCLK to provide sysclk directly*/
	sysclk = data->clk_frequency;

	if (params_width(params) == 24)
			pll_out = sample_rate * 768;
		else
			pll_out = sample_rate * 512;

		ret = snd_soc_dai_set_pll(codec_dai, WM8960_SYSCLK_AUTO, 0,data->clk_frequency, pll_out);
		if (ret) {
			dev_err(dev, "failed to start PLL: %d\n", ret);
			printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
			return ret;
		}

		ret = snd_soc_dai_set_sysclk(codec_dai, WM8960_SYSCLK_AUTO, pll_out, 0);
		if (ret) {
			dev_err(dev, "failed to set SYSCLK: %d\n", ret);
			printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
			return ret;
		}
	printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
	return ret;
}

static int imx_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	struct device *dev = &priv->pdev->dev;
	struct snd_soc_card *card = platform_get_drvdata(priv->pdev);
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
//	struct device *dev = card->dev;
//	int ret;


	data->is_stream_in_use[tx] = false;


	/* Power down PLL to save power*/
	if (data->is_codec_master && !data->is_stream_in_use[!tx]) {
		//printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		snd_soc_dai_set_pll(codec_dai, 0, 0, 0, 0);
		snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF);
	}

	/* We don't need to handle anything if there's no substream running */
		if (!priv->first_stream)
			return 0;

		if (priv->first_stream == substream)
			priv->first_stream = priv->second_stream;
		priv->second_stream = NULL;

		if (!priv->first_stream) {
			/*
			 * Continuously setting FLL would cause playback distortion.
			 * We can fix it just by mute codec after playback.
			 */
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				snd_soc_dai_digital_mute(codec_dai, 1, substream->stream);
		//	printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		}
//	printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
	return 0;
}

static u32 imx_wm8960_rates[] = { 8000, 16000, 32000, 48000 };
static struct snd_pcm_hw_constraint_list imx_wm8960_rate_constraints = {
	.count = ARRAY_SIZE(imx_wm8960_rates),
	.list = imx_wm8960_rates,
};

static int imx_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_dai *codec_dai = rtd->codec_dai;
//	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct imx_priv *priv = &card_priv;
//	struct device *dev = &priv->pdev->dev;
	struct snd_soc_card *card = platform_get_drvdata(priv->pdev);
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
//	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	//struct fsl_sai *sai = dev_get_drvdata(cpu_dai->dev);
	int ret = 0;

//	data->is_stream_opened[tx] = true;

/*	if (data->is_stream_opened[tx] != sai->is_stream_opened[tx] ||
	    data->is_stream_opened[!tx] != sai->is_stream_opened[!tx]) {
		data->is_stream_opened[tx] = false;
		return -EBUSY;
	}
*/
	if (!data->is_codec_master) {
		ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE, &imx_wm8960_rate_constraints);
//		printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		if (ret)
		{
//			printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
			return ret;
		}
								}

	ret = clk_prepare_enable(data->codec_clk);
	//printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
	if (ret) {
		dev_err(card->dev, "Failed to enable MCLK: %d\n", ret);
	//	printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		return ret;
	}
//	printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
	return 0;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_dai *codec_dai = rtd->codec_dai;
	//struct imx_priv *priv = &card_priv;
	//struct device *dev = &priv->pdev->dev;
	struct snd_soc_card *card = rtd->card;//platform_get_drvdata(priv->pdev);
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
//	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	clk_disable_unprepare(data->codec_clk);

//	data->is_stream_opened[tx] = false;
//	printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
	return 0;
}

static struct snd_soc_ops imx_hifi_ops = {
	.hw_params = imx_hifi_hw_params,
	.hw_free = imx_hifi_hw_free,
	.startup   = imx_hifi_startup,
	.shutdown  = imx_hifi_shutdown,
};

static int imx_wm8960_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *codec_dai;
//	struct snd_soc_codec *codec = codec_dai->component;
	struct imx_priv *priv = &card_priv;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	struct device *dev = &priv->pdev->dev;
	int ret;

	data->clk_frequency = clk_get_rate(data->codec_clk);
	rtd = snd_soc_get_pcm_runtime(card, card->dai_link[0].name);
	codec_dai = rtd->codec_dai;
//	snd_soc_component_update_bits(codec, WM8960_IFACE2, 1<<6, 1<<6);
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8960_SYSCLK_MCLK,
				data->clk_frequency, SND_SOC_CLOCK_IN);
		if (ret < 0)
			dev_err(dev, "failed to set sysclk in %s\n", __func__);
	/*
	 * set SAI2_MCLK_DIR to enable codec MCLK
	 */
#if 0
	if (data->gpr)
		regmap_update_bits(data->gpr, 4, 1<<20, 1<<20);

	/*
	 * codec ADCLRC pin configured as GPIO, DACLRC pin is used as a frame
	 * clock for ADCs and DACs
	 */
	snd_soc_component_update_bits(priv->component, WM8960_IFACE2, 1<<6, 1<<6);

	/*
	 * GPIO1 used as headphone detect output
	 */
	snd_soc_component_update_bits(priv->component, WM8960_ADDCTL4, 7<<4, 3<<4);

	/*
	 * Enable headphone jack detect
	 */
	snd_soc_component_update_bits(priv->component, WM8960_ADDCTL2, 1<<6, 1<<6);
	snd_soc_component_update_bits(priv->component, WM8960_ADDCTL2, 1<<5, data->hp_det[1]<<5);
	snd_soc_component_update_bits(priv->component, WM8960_ADDCTL4, 3<<2, data->hp_det[0]<<2);
	snd_soc_component_update_bits(priv->component, WM8960_ADDCTL1, 3, 3);

	/*
	 * route left channel to right channel in default.
	 */
	snd_soc_component_update_bits(priv->component, WM8960_ADDCTL1, 3<<2, 1<<2);

	    snd_soc_write(codec,0x0 ,0x13f);
	    snd_soc_write(codec,0x1 , 0x13f);
		snd_soc_write(codec,0x2 , 0x379);
		snd_soc_write(codec,0x3 , 0x379);
		snd_soc_write(codec, 0x4 , 0x5);
		snd_soc_write(codec,0x5 , 0x0);
		snd_soc_write(codec,0x6 ,0x0);
		snd_soc_write(codec,0x7 , 0x42);
		snd_soc_write(codec, 0x8 , 0x1c4);
		snd_soc_write(codec,0x9 , 0x0);
		snd_soc_write(codec,0xa , 0xd6);//-20dB LDAC
		snd_soc_write(codec,0xb , 0xd6);//-20dB RDAC
		snd_soc_write(codec,0x10 , 0x0);
		snd_soc_write(codec,0x11 , 0x7b);
		snd_soc_write(codec, 0x12 , 0x100);
		snd_soc_write(codec,0x13 , 0x32);
		snd_soc_write(codec,0x14 , 0x0);
		snd_soc_write(codec,0x15 , 0xc3);
		snd_soc_write(codec,0x16 , 0xc3);
		snd_soc_write(codec,0x17 , 0x1d0);//DAC Mono Mix
		snd_soc_write(codec,0x18 , 0x0);
		snd_soc_write(codec,0x19 , 0xfe);//Master clock enabled,MICBIAS power up
		snd_soc_write(codec,0x1a , 0x1fb);
		snd_soc_write(codec,0x1b , 0x0);
		snd_soc_write(codec,0x1c , 0x8);
		snd_soc_write(codec,0x1d , 0x0);
		snd_soc_write(codec,0x20 , 0x108);//LINPUT1 connected to PGA;Connect Left Input PGA to Left Input Boost Mixer
		snd_soc_write(codec,0x21 , 0x108);//RINPUT1 connected to PGA;Connect Right Input PGA to Right Input Boost Mixer
		snd_soc_write(codec,0x22 , 0x100);//Left DAC to Left Output Mixer
		snd_soc_write(codec,0x25 , 0x100);//Right DAC to Right Output Mixer
		snd_soc_write(codec,0x26 , 0x0);
		snd_soc_write(codec,0x27 , 0x0);
		snd_soc_write(codec,0x28 , 0x165);
		snd_soc_write(codec,0x29 , 0x165);
		snd_soc_write(codec,0x2a , 0x40);
		snd_soc_write(codec,0x2b , 0x0);
		snd_soc_write(codec,0x2c , 0x0);
		snd_soc_write(codec,0x2d , 0x50);
		snd_soc_write(codec,0x2e , 0x50);
		snd_soc_write(codec,0x2f , 0x3c);
		snd_soc_write(codec,0x30 , 0x2);
		snd_soc_write(codec,0x31 , 0xf7);//Enable Class D Speaker Outputs,11 = Left and right speakers enabled
		snd_soc_write(codec,0x33 , 0x83);//DC GAIN 000,AC GAIN 011
		snd_soc_write(codec,0x34 , 0x37);
		snd_soc_write(codec,0x35 , 0x86);
		snd_soc_write(codec,0x36 , 0xc2);
		snd_soc_write(codec,0x37 , 0x27);
#endif
	return ret;
}

static int be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{

	struct imx_priv *priv = &card_priv;
	struct snd_interval *rate;
	struct snd_mask *mask;

	if (!priv->asrc_pdev)
		return -EINVAL;

	rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	rate->max = rate->min = priv->asrc_rate;

	mask = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	snd_mask_none(mask);
	snd_mask_set(mask, priv->asrc_format);

	return 0;
}
/*
static struct snd_soc_dai_link imx_wm8960_dai[] = {
	{
		.name = "HiFi",
		.stream_name = "HiFi",
		.codec_dai_name = "wm8960-hifi",
		.ops = &imx_hifi_ops,
	},
	{
		.name = "HiFi-ASRC-FE",
		.stream_name = "HiFi-ASRC-FE",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_pmdown_time = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "HiFi-ASRC-BE",
		.stream_name = "HiFi-ASRC-BE",
		.codec_dai_name = "wm8960-hifi",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.ignore_pmdown_time = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &imx_hifi_ops,
		.be_hw_params_fixup = be_hw_params_fixup,
	},
};
*/
static int imx_wm8960_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *cpu_np = NULL, *codec_np = NULL;// *gpr_np;
	struct platform_device *cpu_pdev;
	struct imx_priv *priv = &card_priv;
	struct i2c_client *codec_dev;
	struct imx_wm8960_data *data;
	struct platform_device *asrc_pdev = NULL;
	struct device_node *asrc_np;
//	struct snd_soc_dai_link_component dlc = { 0 };
//	struct clk *codec_clk = NULL;
//	struct snd_soc_dai *codec_dai;
	struct snd_soc_dai_link_component *dlc;
	u32 width;
	int ret;
	int int_port, ext_port, tmp_port;

	priv->pdev = pdev;
	priv->asrc_pdev = NULL;
/* added 20200810 */
	dlc = devm_kzalloc(&pdev->dev, 9 * sizeof(*dlc), GFP_KERNEL);
		if (!dlc)
			return -ENOMEM;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
		if (!data) {
			ret = -ENOMEM;

			goto fail;
		}

	if (of_property_read_bool(pdev->dev.of_node, "codec-master"))
			data->is_codec_master = true;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;

		goto fail;
	}
#if 1  //增加audmux部分
       if (!strstr(cpu_np->name, "ssi"))
       {

    	   goto audmux_bypass;
       }

        ret = of_property_read_u32(np, "mux-int-port", &int_port);
        if (ret) {
                dev_err(&pdev->dev, "mux-int-port missing or invalid\n");

                goto fail;
        }
        ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
        if (ret) {
                dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");

                goto fail;
        }

        /*
         * The port numbering in the hardware manual starts at 1, while
         * the audmux API expects it starts at 0.
         */
        int_port--;
        ext_port--;

        if (data->is_codec_master) {
        		tmp_port = int_port;
        		int_port = ext_port;
        		ext_port = tmp_port;
        	}

   //     dev_err(&pdev->dev, "audmix int-port:%d,ext-port%d\n",int_port,ext_port);
        ret = imx_audmux_v2_configure_port(ext_port,
                        IMX_AUDMUX_V2_PTCR_SYN |
                        IMX_AUDMUX_V2_PTCR_TFSEL(int_port) |
                        IMX_AUDMUX_V2_PTCR_TCSEL(int_port) |
                        IMX_AUDMUX_V2_PTCR_TFSDIR |
                        IMX_AUDMUX_V2_PTCR_TCLKDIR,
                        IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
        if (ret) {

                dev_err(&pdev->dev, "audmux internal port setup failed\n");

                goto fail;
        }
        imx_audmux_v2_configure_port(int_port,
                        IMX_AUDMUX_V2_PTCR_SYN,
                        IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
        if (ret) {
                dev_err(&pdev->dev, "audmux external port setup failed\n");

                goto fail;
        }

audmux_bypass:

#endif

	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;

		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;

		goto fail;
	}

	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;

		goto fail;
	}
/*
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	if (of_property_read_bool(pdev->dev.of_node, "codec-master"))
		data->is_codec_master = true;
*/

	priv->first_stream = NULL;
	priv->second_stream = NULL;

	data->codec_clk = devm_clk_get(&codec_dev->dev, "mclk");//Changed from "mclk" to NULL-PJ
	if (IS_ERR(data->codec_clk)) {
		ret = PTR_ERR(data->codec_clk);
		dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
//		printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		goto fail;
	}
#if 1
		data->dai[0].cpus = &dlc[0];
		data->dai[0].num_cpus = 1;
		data->dai[0].platforms = &dlc[1];
		data->dai[0].num_platforms = 1;
		data->dai[0].codecs = &dlc[2];
		data->dai[0].num_codecs = 1;

	    data->dai[0].name = "HiFi";
		data->dai[0].stream_name = "HiFi";
		data->dai[0].codecs->dai_name = "wm8960-hifi";
		data->dai[0].codecs->of_node = codec_np;
		data->dai[0].cpus->dai_name = dev_name(&cpu_pdev->dev);
		data->dai[0].platforms->of_node = cpu_np;
		data->dai[0].ops = &imx_hifi_ops;
		data->dai[0].dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF;
		if (data->is_codec_master)
			data->dai[0].dai_fmt |= SND_SOC_DAIFMT_CBM_CFM;
		else
			data->dai[0].dai_fmt |= SND_SOC_DAIFMT_CBS_CFS;

		data->card.num_links = 1;
#if 1
		if (asrc_pdev) {
			data->dai[1].cpus = &dlc[3];
			data->dai[1].num_cpus = 1;
			data->dai[1].platforms = &dlc[4];
			data->dai[1].num_platforms = 1;
			data->dai[1].codecs = &dlc[5];
			data->dai[1].num_codecs = 1;

			data->dai[2].cpus = &dlc[6];
			data->dai[2].num_cpus = 1;
			data->dai[2].platforms = &dlc[7];
			data->dai[2].num_platforms = 1;
			data->dai[2].codecs = &dlc[8];
			data->dai[2].num_codecs = 1;


			data->dai[0].ignore_pmdown_time = 1;
			data->dai[1].name = "HiFi-ASRC-FE";
			data->dai[1].stream_name = "HiFi-ASRC-FE";
			data->dai[1].codecs->name = "snd-soc-dummy";
			data->dai[1].codecs->dai_name = "snd-soc-dummy-dai";
			data->dai[1].cpus->of_node = asrc_np;
			data->dai[1].platforms->of_node = asrc_np;
			data->dai[1].dynamic = 1;
			data->dai[1].ignore_pmdown_time = 1;
			data->dai[1].dpcm_playback = 1;
			data->dai[1].dpcm_capture = 1;
			data->dai[1].dpcm_merged_chan = 1;

			data->dai[2].name = "HiFi-ASRC-BE";
			data->dai[2].stream_name = "HiFi-ASRC-BE";
			data->dai[2].codecs->dai_name = "wm8960";
			data->dai[2].codecs->of_node = codec_np;
			data->dai[2].cpus->dai_name = dev_name(&cpu_pdev->dev);
			data->dai[2].platforms->name = "snd-soc-dummy";
			data->dai[2].ops = &imx_hifi_ops;
			data->dai[2].be_hw_params_fixup = be_hw_params_fixup;
			data->dai[2].no_pcm = 1;
			data->dai[2].ignore_pmdown_time = 1;
			data->dai[2].dpcm_playback = 1;
			data->dai[2].dpcm_capture = 1;
			data->card.num_links = 3;

			ret = of_property_read_u32(asrc_np, "fsl,asrc-rate",
							&priv->asrc_rate);
			if (ret) {
				dev_err(&pdev->dev, "failed to get output rate\n");
				ret = -EINVAL;
				goto fail;
			}

			ret = of_property_read_u32(asrc_np, "fsl,asrc-width", &width);
			if (ret) {
				dev_err(&pdev->dev, "failed to get output rate\n");
				ret = -EINVAL;
				goto fail;
			}

			if (width == 24)
				priv->asrc_format = SNDRV_PCM_FORMAT_S24_LE;
			else
				priv->asrc_format = SNDRV_PCM_FORMAT_S16_LE;
		}
#endif
		    data->card.dev = &pdev->dev;
			ret = snd_soc_of_parse_card_name(&data->card, "model");
			if (ret)
			{
//				printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
				goto fail;
			}
			ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
			if (ret)
			{
//				printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
				goto fail;
			}
//			data->card.owner = THIS_MODULE;
			data->card.dai_link = data->dai;
			data->card.dapm_widgets = imx_wm8960_dapm_widgets;
			data->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8960_dapm_widgets);

			data->card.late_probe = imx_wm8960_late_probe;
#endif
#if 0
	data->codec_clk = codec_clk;
	data->clk_frequency = clk_get_rate(codec_clk);  //added 20200810

	gpr_np = of_parse_phandle(pdev->dev.of_node, "gpr", 0);
        if (gpr_np) {
		data->gpr = syscon_node_to_regmap(gpr_np);
		if (IS_ERR(data->gpr)) {
			ret = PTR_ERR(data->gpr);
			dev_err(&pdev->dev, "failed to get gpr regmap\n");
			goto fail;
		}
	}

	of_property_read_u32_array(pdev->dev.of_node, "hp-det", data->hp_det, 2);

	asrc_np = of_parse_phandle(pdev->dev.of_node, "asrc-controller", 0);
	if (asrc_np) {
		asrc_pdev = of_find_device_by_node(asrc_np);
		priv->asrc_pdev = asrc_pdev;
	}

	data->card.dai_link = imx_wm8960_dai;

	imx_wm8960_dai[0].codec_of_node	= codec_np;
	imx_wm8960_dai[0].cpu_dai_name = dev_name(&cpu_pdev->dev);
	imx_wm8960_dai[0].platform_of_node = cpu_np;

	if (!asrc_pdev) {
		data->card.num_links = 1;
	} else {
		imx_wm8960_dai[1].cpu_of_node = asrc_np;
		imx_wm8960_dai[1].platform_of_node = asrc_np;
		imx_wm8960_dai[2].codec_of_node	= codec_np;
		imx_wm8960_dai[2].cpu_dai_name = dev_name(&cpu_pdev->dev);
		data->card.num_links = 3;

		ret = of_property_read_u32(asrc_np, "fsl,asrc-rate",
				&data->asrc_rate);
		if (ret) {
			dev_err(&pdev->dev, "failed to get output rate\n");
			ret = -EINVAL;
			goto fail;
		}

		ret = of_property_read_u32(asrc_np, "fsl,asrc-width", &width);
		if (ret) {
			dev_err(&pdev->dev, "failed to get output rate\n");
			ret = -EINVAL;
			goto fail;
		}

		if (width == 24)
			data->asrc_format = SNDRV_PCM_FORMAT_S24_LE;
		else
			data->asrc_format = SNDRV_PCM_FORMAT_S16_LE;
	}

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
	{
		goto fail;
	}

	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
		if (ret)
		{
				goto fail;
			}

	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_wm8960_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8960_dapm_widgets);


	data->card.late_probe = imx_wm8960_late_probe;
#endif
	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
		if (ret) {
			dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
//			printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
			goto fail;
		}
/*
	ret = snd_soc_register_card(&data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		goto fail;
	}
*/
	priv->snd_card = data->card.snd_card;
/*
	priv->hp_set_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "hp-det-gpios", 0,
			(enum of_gpio_flags *)&priv->hp_active_low);
	if (IS_ERR(ERR_PTR(priv->hp_set_gpio)))
	{
		printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		goto fail;
	}

	priv->headset_kctl = snd_kctl_jack_new("Headset", NULL);
	ret = snd_ctl_add(data->card.snd_card, priv->headset_kctl);
	if (ret)
	{
		printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
		goto fail;
	}
*/
	ret = imx_wm8960_gpio_init(&data->card);
	//printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);

	if (gpio_is_valid(priv->hp_set_gpio)) {
		ret = driver_create_file(pdev->dev.driver, &driver_attr_headphone);
		if (ret) {
			dev_err(&pdev->dev, "create hp attr failed (%d)\n", ret);
//			printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
			goto fail;
		}
		ret = driver_create_file(pdev->dev.driver, &driver_attr_micphone);
		if (ret) {
			dev_err(&pdev->dev, "create mic attr failed (%d)\n", ret);
//			printk(KERN_ERR "Yao-log: ---%s,%d!---\n", __FUNCTION__,__LINE__);
			goto fail;
		}
	}

	printk("imx_wm8960_probe  probe success\n");
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static const struct of_device_id imx_wm8960_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-wm8960", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8960_dt_ids);

static struct platform_driver imx_wm8960_driver = {
	.driver = {
		.name = "imx-wm8960",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8960_dt_ids,
	},
	.probe = imx_wm8960_probe,
};
module_platform_driver(imx_wm8960_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX WM8960 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8960");
