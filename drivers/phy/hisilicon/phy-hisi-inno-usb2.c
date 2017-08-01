/*
 * HiSilicon INNO USB2 PHY Driver.
 *
 * Copyright (c) 2016-2017 HiSilicon Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define INNO_PHY_PORT_NUM	2
#define REF_CLK_STABLE_TIME	100	/* unit:us */
#define UTMI_CLK_STABLE_TIME	200	/* unit:us */
#define TEST_CLK_STABLE_TIME	2	/* unit:ms */
#define PHY_CLK_STABLE_TIME	2	/* unit:ms */
#define UTMI_RST_COMPLETE_TIME	2	/* unit:ms */
#define TEST_RST_COMPLETE_TIME	100	/* unit:us */
#define POR_RST_COMPLETE_TIME	300	/* unit:us */
#define PHY_TEST_DATA	GENMASK(7, 0)
#define PHY_TEST_ADDR	GENMASK(15, 8)
#define PHY_TEST_PORT	GENMASK(18, 16)
#define PHY_TEST_WREN	BIT(21)
#define PHY_TEST_CLK	BIT(22)	/* rising edge active */
#define PHY_TEST_RST	BIT(23)	/* low active */
#define PHY_CLK_ENABLE	BIT(2)

struct hisi_inno_phy_priv {
	struct regmap *syscon;
	u32 reg;
	struct clk *ref_clk;
	struct reset_control *por_rst;
	struct reset_control *test_rst;
	struct reset_control *utmi_rst[INNO_PHY_PORT_NUM];
	u32 port_num;
};

static void hisi_inno_phy_write_reg(struct regmap *syscon,
			u32 reg, u8 port, u32 addr, u32 data)
{
	u32 value;

	value = (data & PHY_TEST_DATA)
		| ((addr << 8) & PHY_TEST_ADDR)
		| ((port << 16) & PHY_TEST_PORT)
		| PHY_TEST_WREN | PHY_TEST_RST;
	regmap_write(syscon, reg, value);
	value |= PHY_TEST_CLK;
	regmap_write(syscon, reg, value);
	value &= ~PHY_TEST_CLK;
	regmap_write(syscon, reg, value);
}

static void hisi_inno_phy_setup(struct hisi_inno_phy_priv *priv)
{
	/* The phy clk is controlled by the port0 register 0x06. */
	hisi_inno_phy_write_reg(priv->syscon,
			priv->reg, 0, 0x06, PHY_CLK_ENABLE);
	msleep(PHY_CLK_STABLE_TIME);
}

static int hisi_inno_phy_init(struct phy *phy)
{
	struct hisi_inno_phy_priv *priv = phy_get_drvdata(phy);
	int ret, i;

	ret = clk_prepare_enable(priv->ref_clk);
	if (ret)
		return ret;
	udelay(REF_CLK_STABLE_TIME);

	if (priv->test_rst) {
		reset_control_deassert(priv->test_rst);
		udelay(TEST_RST_COMPLETE_TIME);
	}

	reset_control_deassert(priv->por_rst);
	udelay(POR_RST_COMPLETE_TIME);

	/* config phy clk and phy eye diagram */
	hisi_inno_phy_setup(priv);

	for (i = 0; i < priv->port_num; i++) {
		reset_control_deassert(priv->utmi_rst[i]);
		udelay(UTMI_RST_COMPLETE_TIME);
	}

	return 0;
}

static void hisi_inno_phy_disable(struct phy *phy)
{
	struct hisi_inno_phy_priv *priv = phy_get_drvdata(phy);
	int i;

	for (i = 0; i < priv->port_num; i++)
		reset_control_assert(priv->utmi_rst[i]);

	reset_control_assert(priv->por_rst);
	if (priv->test_rst)
		reset_control_assert(priv->test_rst);
	clk_disable_unprepare(priv->ref_clk);
}

static int hisi_inno_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy *phy;
	struct hisi_inno_phy_priv *priv;
	struct device_node *node = dev->of_node;
	int ret, i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->syscon = syscon_regmap_lookup_by_phandle(node,
			"hisilicon,peripheral-syscon");
	if (IS_ERR(priv->syscon)) {
		dev_err(dev, "no hisilicon,peripheral-syscon\n");
		return PTR_ERR(priv->syscon);
	}

	ret = device_property_read_u32(dev,
			"hisilicon,phycon-reg", &priv->reg);
	if (ret)
		return ret;

	priv->ref_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->ref_clk))
		return PTR_ERR(priv->ref_clk);

	priv->por_rst = devm_reset_control_get_exclusive(dev, "power_on");
	if (IS_ERR(priv->por_rst))
		return PTR_ERR(priv->por_rst);

	ret = device_property_read_u32(dev,
					"hisilicon,port-num", &priv->port_num);
	if (ret)
		return ret;

	if (priv->port_num <= 0
		|| priv->port_num > INNO_PHY_PORT_NUM)
		return -EINVAL;

	for (i = 0; i < priv->port_num; i++) {
		char id[6];

		snprintf(id, 6, "utmi%1d\n", i);
		priv->utmi_rst[i] = devm_reset_control_get_exclusive(dev, id);
		if (IS_ERR(priv->utmi_rst[i]))
			return PTR_ERR(priv->utmi_rst[i]);
	}

	priv->test_rst = devm_reset_control_get_optional_exclusive(dev, "test");
	if (IS_ERR(priv->test_rst))
		return PTR_ERR(priv->test_rst);

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	platform_set_drvdata(pdev, phy);
	phy_set_drvdata(phy, priv);

	return hisi_inno_phy_init(phy);
}

static int hisi_inno_phy_remove(struct platform_device *pdev)
{
	struct phy *phy = platform_get_drvdata(pdev);

	hisi_inno_phy_disable(phy);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hisi_inno_phy_suspend(struct device *dev)
{
	struct phy *phy = dev_get_drvdata(dev);

	hisi_inno_phy_disable(phy);

	return 0;
}

static int hisi_inno_phy_resume(struct device *dev)
{
	struct phy *phy = dev_get_drvdata(dev);

	return hisi_inno_phy_init(phy);
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops hisi_inno_phy_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hisi_inno_phy_suspend, hisi_inno_phy_resume)
};

static const struct of_device_id hisi_inno_phy_of_match[] = {
	{.compatible = "hisilicon,inno-usb2-phy",},
	{.compatible = "hisilicon,hi3798cv200-usb2-phy",},
	{ },
};
MODULE_DEVICE_TABLE(of, hisi_inno_phy_of_match);

static struct platform_driver hisi_inno_phy_driver = {
	.probe	= hisi_inno_phy_probe,
	.remove = hisi_inno_phy_remove,
	.driver = {
		.name	= "hisi-inno-phy",
		.of_match_table	= hisi_inno_phy_of_match,
		.pm    = &hisi_inno_phy_pm_ops,
	}
};
module_platform_driver(hisi_inno_phy_driver);

MODULE_DESCRIPTION("HiSilicon INNO USB2 PHY Driver");
MODULE_LICENSE("GPL v2");
