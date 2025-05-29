#ifndef VIS4EARTH_GRAPH_VISER_GEOGRAPHIC_REGIONS_H
#define VIS4EARTH_GRAPH_VISER_GEOGRAPHIC_REGIONS_H

#include <vector>
#include <string>
#include <osg/Vec3>

namespace VIS4Earth {

// 地理区域结构定义
struct GeographicRegion {
    int regionId;
    float minLat, maxLat, minLon, maxLon;
    std::string regionName;
    osg::Vec3 centerPoint;
    std::vector<std::string> nodeIds;
    int totalConnections = 0;
    float totalWeight = 0.0f;
};

// LOD 0: 全球主要国家级别 (约50个区域)
static const std::vector<GeographicRegion> LOD0_REGIONS = {
    // 北美洲国家
    {0, 60, 72, -180, -140, "Alaska", osg::Vec3(65, -160, 0), {}, 0, 0},
    {1, 45, 60, -140, -50, "Canada", osg::Vec3(55, -100, 0), {}, 0, 0},
    {2, 25, 49, -125, -65, "USA", osg::Vec3(40, -95, 0), {}, 0, 0},
    {3, 15, 33, -118, -80, "Mexico", osg::Vec3(24, -99, 0), {}, 0, 0},
    {4, 8, 20, -95, -75, "Central_America", osg::Vec3(15, -85, 0), {}, 0, 0},
    
    // 南美洲国家
    {5, 8, 15, -85, -55, "Colombia_Venezuela", osg::Vec3(8, -70, 0), {}, 0, 0},
    {6, -20, 8, -80, -35, "Brazil", osg::Vec3(-10, -55, 0), {}, 0, 0},
    {7, -35, -15, -75, -53, "Argentina", osg::Vec3(-25, -64, 0), {}, 0, 0},
    {8, -25, -10, -70, -50, "Chile_Peru", osg::Vec3(-18, -60, 0), {}, 0, 0},
    {9, -20, 8, -90, -80, "Ecuador_Bolivia", osg::Vec3(-5, -85, 0), {}, 0, 0},
    
    // 欧洲国家
    {10, 55, 72, -10, 30, "Nordic_Countries", osg::Vec3(62, 10, 0), {}, 0, 0},
    {11, 49, 60, -5, 15, "UK_France", osg::Vec3(52, 2, 0), {}, 0, 0},
    {12, 45, 55, 5, 15, "Germany_Netherlands", osg::Vec3(50, 10, 0), {}, 0, 0},
    {13, 35, 50, 10, 25, "Italy_Switzerland", osg::Vec3(42, 12, 0), {}, 0, 0},
    {14, 40, 50, 15, 30, "Eastern_Europe", osg::Vec3(45, 22, 0), {}, 0, 0},
    {15, 45, 60, 20, 50, "Russia_West", osg::Vec3(55, 35, 0), {}, 0, 0},
    {16, 35, 45, 25, 45, "Turkey_Greece", osg::Vec3(39, 35, 0), {}, 0, 0},
    {17, 35, 45, -10, 10, "Spain_Portugal", osg::Vec3(40, 0, 0), {}, 0, 0},
    
    // 非洲国家
    {18, 20, 37, -20, 40, "North_Africa", osg::Vec3(28, 10, 0), {}, 0, 0},
    {19, -5, 20, -20, 20, "West_Africa", osg::Vec3(8, 0, 0), {}, 0, 0},
    {20, -5, 15, 20, 50, "East_Africa", osg::Vec3(5, 35, 0), {}, 0, 0},
    {21, -35, -5, 10, 35, "Southern_Africa", osg::Vec3(-20, 25, 0), {}, 0, 0},
    
    // 中东国家
    {22, 25, 40, 35, 65, "Middle_East", osg::Vec3(30, 50, 0), {}, 0, 0},
    {23, 20, 40, 45, 65, "Iran_Afghanistan", osg::Vec3(32, 55, 0), {}, 0, 0},
    
    // 南亚国家
    {24, 8, 37, 65, 80, "India_Pakistan", osg::Vec3(25, 75, 0), {}, 0, 0},
    {25, 20, 30, 80, 90, "Nepal_Bangladesh", osg::Vec3(25, 85, 0), {}, 0, 0},
    {26, 0, 10, 72, 82, "Sri_Lanka_Maldives", osg::Vec3(8, 77, 0), {}, 0, 0},
    
    // 东南亚国家
    {27, -10, 8, 95, 115, "Indonesia", osg::Vec3(-2, 105, 0), {}, 0, 0},
    {28, 0, 25, 95, 110, "Thailand_Myanmar", osg::Vec3(15, 100, 0), {}, 0, 0},
    {29, 10, 25, 110, 125, "Vietnam_Philippines", osg::Vec3(15, 115, 0), {}, 0, 0},
    {30, 0, 8, 100, 120, "Malaysia_Singapore", osg::Vec3(4, 110, 0), {}, 0, 0},
    
    // 东亚国家
    {31, 18, 25, 110, 125, "South_China", osg::Vec3(22, 115, 0), {}, 0, 0},
    {32, 25, 40, 105, 125, "Central_China", osg::Vec3(35, 115, 0), {}, 0, 0},
    {33, 40, 54, 110, 135, "North_China", osg::Vec3(45, 120, 0), {}, 0, 0},
    {34, 30, 46, 125, 145, "Japan", osg::Vec3(36, 138, 0), {}, 0, 0},
    {35, 33, 43, 124, 132, "Korea", osg::Vec3(37, 128, 0), {}, 0, 0},
    {36, 40, 50, 105, 125, "Mongolia", osg::Vec3(46, 105, 0), {}, 0, 0},
    
    // 俄罗斯各部分
    {37, 45, 70, 30, 60, "Russia_Central", osg::Vec3(55, 45, 0), {}, 0, 0},
    {38, 50, 70, 60, 100, "Russia_Siberia_West", osg::Vec3(60, 80, 0), {}, 0, 0},
    {39, 50, 70, 100, 140, "Russia_Siberia_East", osg::Vec3(60, 120, 0), {}, 0, 0},
    {40, 45, 65, 140, 180, "Russia_Far_East", osg::Vec3(55, 160, 0), {}, 0, 0},
    
    // 中亚国家
    {41, 35, 50, 50, 80, "Central_Asia", osg::Vec3(42, 65, 0), {}, 0, 0},
    
    // 大洋洲
    {42, -45, -10, 110, 155, "Australia", osg::Vec3(-25, 135, 0), {}, 0, 0},
    {43, -50, -30, 165, 180, "New_Zealand", osg::Vec3(-40, 175, 0), {}, 0, 0},
    {44, -25, 25, 140, 180, "Pacific_Islands", osg::Vec3(0, 160, 0), {}, 0, 0},
    
    // 北极地区
    {45, 70, 90, -180, 180, "Arctic", osg::Vec3(80, 0, 0), {}, 0, 0},
    
    // 南极地区
    {46, -90, -60, -180, 180, "Antarctica", osg::Vec3(-75, 0, 0), {}, 0, 0}
};

// LOD 1: 国家内部大区域划分 (约150个区域)
static const std::vector<GeographicRegion> LOD1_REGIONS = {
    // 美国内部分区
    {0, 45, 49, -125, -110, "USA_Northwest", osg::Vec3(47, -118, 0), {}, 0, 0},
    {1, 40, 45, -125, -110, "USA_California_North", osg::Vec3(42, -120, 0), {}, 0, 0},
    {2, 32, 40, -125, -110, "USA_California_South", osg::Vec3(36, -118, 0), {}, 0, 0},
    {3, 25, 37, -110, -95, "USA_Southwest", osg::Vec3(32, -105, 0), {}, 0, 0},
    {4, 37, 45, -110, -95, "USA_Mountain", osg::Vec3(41, -105, 0), {}, 0, 0},
    {5, 25, 37, -95, -80, "USA_South", osg::Vec3(32, -87, 0), {}, 0, 0},
    {6, 37, 45, -95, -80, "USA_Midwest", osg::Vec3(41, -87, 0), {}, 0, 0},
    {7, 37, 45, -80, -65, "USA_Northeast", osg::Vec3(41, -72, 0), {}, 0, 0},
    {8, 25, 37, -85, -75, "USA_Florida", osg::Vec3(28, -82, 0), {}, 0, 0},
    {9, 60, 72, -180, -140, "USA_Alaska", osg::Vec3(65, -160, 0), {}, 0, 0},
    {10, 18, 28, -165, -154, "USA_Hawaii", osg::Vec3(21, -158, 0), {}, 0, 0},
    
    // 加拿大内部分区
    {11, 45, 60, -140, -120, "Canada_West", osg::Vec3(55, -130, 0), {}, 0, 0},
    {12, 45, 60, -120, -100, "Canada_Prairie", osg::Vec3(55, -110, 0), {}, 0, 0},
    {13, 45, 60, -100, -80, "Canada_Central", osg::Vec3(55, -90, 0), {}, 0, 0},
    {14, 45, 60, -80, -50, "Canada_East", osg::Vec3(55, -65, 0), {}, 0, 0},
    {15, 60, 85, -140, -60, "Canada_Arctic", osg::Vec3(70, -100, 0), {}, 0, 0},
    
    // 中国内部分区
    {16, 18, 25, 105, 115, "China_Southwest", osg::Vec3(22, 110, 0), {}, 0, 0},
    {17, 25, 35, 100, 115, "China_Central", osg::Vec3(30, 108, 0), {}, 0, 0},
    {18, 35, 42, 105, 125, "China_North", osg::Vec3(38, 115, 0), {}, 0, 0},
    {19, 40, 54, 115, 135, "China_Northeast", osg::Vec3(45, 125, 0), {}, 0, 0},
    {20, 25, 40, 115, 125, "China_East", osg::Vec3(32, 120, 0), {}, 0, 0},
    {21, 30, 42, 75, 105, "China_West", osg::Vec3(36, 90, 0), {}, 0, 0},
    {22, 18, 25, 108, 118, "China_South", osg::Vec3(22, 113, 0), {}, 0, 0},
    
    // 俄罗斯内部分区
    {23, 55, 65, 30, 50, "Russia_Moscow", osg::Vec3(60, 40, 0), {}, 0, 0},
    {24, 45, 55, 30, 50, "Russia_South", osg::Vec3(50, 40, 0), {}, 0, 0},
    {25, 55, 70, 50, 80, "Russia_Urals", osg::Vec3(62, 65, 0), {}, 0, 0},
    {26, 50, 65, 80, 110, "Russia_Siberia_Central", osg::Vec3(58, 95, 0), {}, 0, 0},
    {27, 50, 65, 110, 140, "Russia_Siberia_East", osg::Vec3(58, 125, 0), {}, 0, 0},
    {28, 45, 60, 140, 170, "Russia_Far_East", osg::Vec3(55, 155, 0), {}, 0, 0},
    {29, 65, 85, 30, 180, "Russia_Arctic", osg::Vec3(75, 105, 0), {}, 0, 0},
    
    // 巴西内部分区
    {30, -5, 5, -75, -45, "Brazil_North", osg::Vec3(0, -60, 0), {}, 0, 0},
    {31, -15, -5, -60, -35, "Brazil_Northeast", osg::Vec3(-10, -45, 0), {}, 0, 0},
    {32, -25, -15, -55, -40, "Brazil_Southeast", osg::Vec3(-20, -47, 0), {}, 0, 0},
    {33, -25, -15, -65, -55, "Brazil_South", osg::Vec3(-20, -60, 0), {}, 0, 0},
    {34, -15, -5, -75, -60, "Brazil_Central", osg::Vec3(-10, -67, 0), {}, 0, 0},
    
    // 印度内部分区
    {35, 20, 30, 65, 75, "India_North", osg::Vec3(25, 70, 0), {}, 0, 0},
    {36, 15, 25, 70, 80, "India_Central", osg::Vec3(20, 75, 0), {}, 0, 0},
    {37, 8, 20, 70, 80, "India_South", osg::Vec3(15, 75, 0), {}, 0, 0},
    {38, 20, 30, 80, 90, "India_East", osg::Vec3(25, 85, 0), {}, 0, 0},
    {39, 15, 25, 65, 75, "India_West", osg::Vec3(20, 70, 0), {}, 0, 0},
    
    // 澳大利亚内部分区
    {40, -20, -10, 110, 130, "Australia_North", osg::Vec3(-15, 120, 0), {}, 0, 0},
    {41, -35, -20, 110, 130, "Australia_Central", osg::Vec3(-27, 120, 0), {}, 0, 0},
    {42, -45, -35, 110, 130, "Australia_South", osg::Vec3(-40, 120, 0), {}, 0, 0},
    {43, -35, -10, 130, 155, "Australia_East", osg::Vec3(-22, 142, 0), {}, 0, 0},
    {44, -35, -20, 110, 125, "Australia_West", osg::Vec3(-27, 118, 0), {}, 0, 0},
    
    // 德国内部分区
    {45, 50, 55, 5, 15, "Germany_North", osg::Vec3(52, 10, 0), {}, 0, 0},
    {46, 47, 52, 5, 15, "Germany_Central", osg::Vec3(50, 10, 0), {}, 0, 0},
    {47, 45, 50, 5, 15, "Germany_South", osg::Vec3(48, 11, 0), {}, 0, 0},
    
    // 法国内部分区
    {48, 47, 51, -5, 8, "France_North", osg::Vec3(49, 2, 0), {}, 0, 0},
    {49, 43, 47, -5, 8, "France_South", osg::Vec3(45, 2, 0), {}, 0, 0},
    
    // 英国内部分区
    {50, 53, 60, -8, 2, "UK_Scotland", osg::Vec3(56, -3, 0), {}, 0, 0},
    {51, 50, 55, -5, 2, "UK_England", osg::Vec3(52, -1, 0), {}, 0, 0},
    {52, 50, 55, -5, -2, "UK_Wales", osg::Vec3(52, -3, 0), {}, 0, 0},
    {53, 53, 56, -8, -5, "UK_Ireland", osg::Vec3(54, -6, 0), {}, 0, 0},
    
    // 日本内部分区
    {54, 35, 46, 129, 146, "Japan_Honshu", osg::Vec3(36, 138, 0), {}, 0, 0},
    {55, 31, 34, 129, 132, "Japan_Kyushu", osg::Vec3(32, 131, 0), {}, 0, 0},
    {56, 42, 46, 140, 146, "Japan_Hokkaido", osg::Vec3(43, 143, 0), {}, 0, 0},
    {57, 33, 35, 132, 135, "Japan_Shikoku", osg::Vec3(34, 134, 0), {}, 0, 0},
    {58, 24, 26, 122, 132, "Japan_Okinawa", osg::Vec3(26, 128, 0), {}, 0, 0},
    
    // 其他主要国家的区域划分...
    // 意大利分区
    {59, 45, 47, 6, 15, "Italy_North", osg::Vec3(46, 10, 0), {}, 0, 0},
    {60, 40, 45, 8, 18, "Italy_Central", osg::Vec3(42, 13, 0), {}, 0, 0},
    {61, 35, 42, 12, 20, "Italy_South", osg::Vec3(39, 16, 0), {}, 0, 0},
    
    // 西班牙分区
    {62, 40, 44, -10, 0, "Spain_North", osg::Vec3(42, -5, 0), {}, 0, 0},
    {63, 35, 42, -10, 5, "Spain_South", osg::Vec3(38, -2, 0), {}, 0, 0},
    
    // 墨西哥分区
    {64, 20, 33, -118, -95, "Mexico_North", osg::Vec3(26, -106, 0), {}, 0, 0},
    {65, 15, 22, -105, -87, "Mexico_Central", osg::Vec3(19, -96, 0), {}, 0, 0},
    {66, 14, 18, -95, -87, "Mexico_South", osg::Vec3(16, -91, 0), {}, 0, 0},
    
    // 阿根廷分区
    {67, -25, -15, -75, -53, "Argentina_North", osg::Vec3(-20, -64, 0), {}, 0, 0},
    {68, -40, -25, -72, -53, "Argentina_Central", osg::Vec3(-32, -62, 0), {}, 0, 0},
    {69, -55, -40, -75, -53, "Argentina_South", osg::Vec3(-47, -64, 0), {}, 0, 0},
    
    // 其他区域...
    {70, -20, 8, -80, -35, "Brazil_Amazon", osg::Vec3(-5, -57, 0), {}, 0, 0}
};

// LOD 2: 省份/州级别 (约150个区域) - 重新设计为更大的覆盖范围
static const std::vector<GeographicRegion> LOD2_REGIONS = {
    // 美国各州区域 (扩大范围)
    {0, 40, 45, -80, -66, "USA_Northeast_States", osg::Vec3(42.5, -73, 0), {}, 0, 0}, // 纽约、新英格兰
    {1, 32, 40, -80, -75, "USA_MidAtlantic", osg::Vec3(36, -77, 0), {}, 0, 0}, // 宾州、弗吉尼亚
    {2, 25, 35, -87, -75, "USA_Southeast", osg::Vec3(30, -81, 0), {}, 0, 0}, // 佛罗里达、乔治亚
    {3, 35, 42, -90, -80, "USA_Ohio_Valley", osg::Vec3(38, -85, 0), {}, 0, 0}, // 俄亥俄、肯塔基
    {4, 40, 48, -90, -80, "USA_Great_Lakes", osg::Vec3(44, -85, 0), {}, 0, 0}, // 密歇根、威斯康星
    {5, 35, 42, -100, -90, "USA_Central_Plains", osg::Vec3(38, -95, 0), {}, 0, 0}, // 堪萨斯、密苏里
    {6, 25, 35, -100, -90, "USA_South_Central", osg::Vec3(30, -95, 0), {}, 0, 0}, // 德克萨斯、路易斯安那
    {7, 40, 50, -110, -100, "USA_Northern_Plains", osg::Vec3(45, -105, 0), {}, 0, 0}, // 北达科他、蒙大拿
    {8, 30, 40, -110, -100, "USA_Southern_Rockies", osg::Vec3(35, -105, 0), {}, 0, 0}, // 科罗拉多、新墨西哥
    {9, 40, 50, -125, -110, "USA_Northwest_States", osg::Vec3(45, -118, 0), {}, 0, 0}, // 华盛顿、俄勒冈
    {10, 32, 40, -125, -110, "USA_California", osg::Vec3(36, -118, 0), {}, 0, 0}, // 加利福尼亚
    {11, 60, 72, -180, -140, "USA_Alaska", osg::Vec3(65, -160, 0), {}, 0, 0}, // 阿拉斯加
    {12, 18, 28, -165, -154, "USA_Hawaii", osg::Vec3(21, -158, 0), {}, 0, 0}, // 夏威夷
    
    // 加拿大各省区域
    {13, 48, 60, -140, -120, "Canada_British_Columbia", osg::Vec3(54, -130, 0), {}, 0, 0},
    {14, 49, 60, -120, -100, "Canada_Prairie_Provinces", osg::Vec3(54, -110, 0), {}, 0, 0},
    {15, 45, 57, -95, -74, "Canada_Ontario", osg::Vec3(51, -84, 0), {}, 0, 0},
    {16, 45, 55, -80, -57, "Canada_Quebec", osg::Vec3(50, -68, 0), {}, 0, 0},
    {17, 43, 50, -67, -53, "Canada_Maritime", osg::Vec3(46, -60, 0), {}, 0, 0},
    {18, 60, 85, -140, -60, "Canada_Northern_Territories", osg::Vec3(70, -100, 0), {}, 0, 0},
    
    // 中国各省区域
    {19, 39, 42, 115, 120, "China_Beijing_Tianjin", osg::Vec3(40, 117, 0), {}, 0, 0},
    {20, 30, 35, 118, 122, "China_Yangtze_Delta", osg::Vec3(32, 120, 0), {}, 0, 0}, // 上海、江苏
    {21, 22, 26, 110, 118, "China_Pearl_River_Delta", osg::Vec3(24, 114, 0), {}, 0, 0}, // 广东
    {22, 28, 33, 103, 108, "China_Sichuan_Chongqing", osg::Vec3(30, 105, 0), {}, 0, 0},
    {23, 29, 33, 110, 116, "China_Central_Provinces", osg::Vec3(31, 113, 0), {}, 0, 0}, // 湖南、湖北
    {24, 34, 40, 110, 120, "China_North_Central", osg::Vec3(37, 115, 0), {}, 0, 0}, // 河南、山东
    {25, 43, 54, 115, 135, "China_Northeast", osg::Vec3(47, 125, 0), {}, 0, 0}, // 东北三省
    {26, 35, 42, 100, 110, "China_Northwest", osg::Vec3(38, 105, 0), {}, 0, 0}, // 陕西、甘肃
    {27, 25, 30, 100, 110, "China_Southwest", osg::Vec3(27, 105, 0), {}, 0, 0}, // 云南、贵州
    {28, 35, 50, 75, 100, "China_Western_Regions", osg::Vec3(42, 87, 0), {}, 0, 0}, // 新疆、西藏
    
    // 俄罗斯各地区
    {29, 55, 68, 30, 50, "Russia_Central_Federal", osg::Vec3(60, 40, 0), {}, 0, 0}, // 莫斯科地区
    {30, 45, 55, 30, 50, "Russia_Southern_Federal", osg::Vec3(50, 40, 0), {}, 0, 0},
    {31, 55, 68, 50, 80, "Russia_Ural_Federal", osg::Vec3(62, 65, 0), {}, 0, 0},
    {32, 50, 68, 80, 110, "Russia_Siberian_Federal", osg::Vec3(60, 95, 0), {}, 0, 0},
    {33, 45, 65, 110, 140, "Russia_Far_Eastern_Federal", osg::Vec3(55, 125, 0), {}, 0, 0},
    {34, 68, 85, 30, 180, "Russia_Arctic_Regions", osg::Vec3(75, 105, 0), {}, 0, 0},
    
    // 巴西各州
    {35, -5, 5, -75, -45, "Brazil_Amazon_States", osg::Vec3(0, -60, 0), {}, 0, 0},
    {36, -15, -5, -60, -35, "Brazil_Northeast_States", osg::Vec3(-10, -47, 0), {}, 0, 0},
    {37, -25, -15, -55, -40, "Brazil_Southeast_States", osg::Vec3(-20, -47, 0), {}, 0, 0}, // 圣保罗、里约
    {38, -35, -25, -65, -45, "Brazil_Southern_States", osg::Vec3(-30, -55, 0), {}, 0, 0},
    {39, -20, -5, -75, -60, "Brazil_Central_West", osg::Vec3(-12, -67, 0), {}, 0, 0},
    
    // 印度各邦
    {40, 28, 35, 75, 80, "India_Northern_States", osg::Vec3(31, 77, 0), {}, 0, 0}, // 德里、旁遮普
    {41, 20, 28, 68, 78, "India_Western_States", osg::Vec3(24, 73, 0), {}, 0, 0}, // 马哈拉施特拉
    {42, 20, 28, 78, 88, "India_Central_States", osg::Vec3(24, 83, 0), {}, 0, 0},
    {43, 20, 28, 88, 95, "India_Eastern_States", osg::Vec3(24, 91, 0), {}, 0, 0}, // 西孟加拉
    {44, 8, 20, 75, 88, "India_Southern_States", osg::Vec3(14, 81, 0), {}, 0, 0}, // 泰米尔纳德
    
    // 澳大利亚各州
    {45, -29, -10, 113, 129, "Australia_Western_Australia", osg::Vec3(-20, 121, 0), {}, 0, 0},
    {46, -38, -26, 129, 142, "Australia_South_Australia", osg::Vec3(-32, 135, 0), {}, 0, 0},
    {47, -39, -34, 140, 150, "Australia_Victoria", osg::Vec3(-36, 145, 0), {}, 0, 0},
    {48, -37, -28, 140, 154, "Australia_New_South_Wales", osg::Vec3(-32, 147, 0), {}, 0, 0},
    {49, -29, -10, 138, 154, "Australia_Queensland", osg::Vec3(-20, 146, 0), {}, 0, 0},
    {50, -44, -39, 144, 149, "Australia_Tasmania", osg::Vec3(-41, 146, 0), {}, 0, 0},
    {51, -26, -10, 129, 138, "Australia_Northern_Territory", osg::Vec3(-18, 133, 0), {}, 0, 0},
    
    // 德国各州
    {52, 53, 56, 8, 15, "Germany_Northern_States", osg::Vec3(54, 11, 0), {}, 0, 0},
    {53, 49, 53, 6, 15, "Germany_Central_States", osg::Vec3(51, 10, 0), {}, 0, 0},
    {54, 47, 50, 7, 13, "Germany_Southern_States", osg::Vec3(48, 10, 0), {}, 0, 0},
    
    // 法国大区
    {55, 48, 51, -5, 8, "France_Northern_Regions", osg::Vec3(49, 2, 0), {}, 0, 0},
    {56, 43, 48, -5, 8, "France_Southern_Regions", osg::Vec3(45, 2, 0), {}, 0, 0},
    
    // 英国各地区
    {57, 53, 61, -8, 2, "UK_Scotland_Northern_Ireland", osg::Vec3(56, -3, 0), {}, 0, 0},
    {58, 50, 56, -5, 2, "UK_England_Wales", osg::Vec3(52, -2, 0), {}, 0, 0},
    
    // 意大利大区
    {59, 45, 47, 6, 15, "Italy_Northern_Regions", osg::Vec3(46, 10, 0), {}, 0, 0},
    {60, 40, 45, 8, 18, "Italy_Central_Regions", osg::Vec3(42, 13, 0), {}, 0, 0},
    {61, 35, 42, 12, 20, "Italy_Southern_Regions", osg::Vec3(39, 16, 0), {}, 0, 0},
    
    // 西班牙大区
    {62, 40, 44, -10, 4, "Spain_Northern_Regions", osg::Vec3(42, -3, 0), {}, 0, 0},
    {63, 35, 42, -10, 4, "Spain_Southern_Regions", osg::Vec3(38, -3, 0), {}, 0, 0},
    
    // 日本各地区
    {64, 35, 42, 135, 142, "Japan_Kanto_Chubu", osg::Vec3(38, 139, 0), {}, 0, 0}, // 关东、中部
    {65, 33, 37, 130, 137, "Japan_Kansai_Chugoku", osg::Vec3(35, 134, 0), {}, 0, 0}, // 关西、中国
    {66, 31, 35, 129, 132, "Japan_Kyushu_Okinawa", osg::Vec3(32, 130, 0), {}, 0, 0},
    {67, 40, 46, 139, 146, "Japan_Tohoku_Hokkaido", osg::Vec3(42, 142, 0), {}, 0, 0},
    
    // 韩国各道
    {68, 36, 39, 126, 129, "Korea_Seoul_Capital", osg::Vec3(37, 127, 0), {}, 0, 0},
    {69, 33, 37, 124, 130, "Korea_Southern_Provinces", osg::Vec3(35, 127, 0), {}, 0, 0},
    
    // 墨西哥各州
    {70, 25, 33, -118, -95, "Mexico_Northern_States", osg::Vec3(29, -106, 0), {}, 0, 0},
    {71, 17, 25, -105, -87, "Mexico_Central_States", osg::Vec3(21, -96, 0), {}, 0, 0},
    {72, 14, 20, -98, -87, "Mexico_Southern_States", osg::Vec3(17, -92, 0), {}, 0, 0},
    
    // 阿根廷各省
    {73, -30, -20, -70, -53, "Argentina_Northern_Provinces", osg::Vec3(-25, -61, 0), {}, 0, 0},
    {74, -42, -30, -72, -53, "Argentina_Central_Provinces", osg::Vec3(-36, -62, 0), {}, 0, 0},
    {75, -55, -42, -75, -53, "Argentina_Patagonia", osg::Vec3(-48, -64, 0), {}, 0, 0},
    
    // 其他重要区域
    {76, 50, 60, 5, 15, "Benelux_Region", osg::Vec3(52, 5, 0), {}, 0, 0}, // 荷兰、比利时
    {77, 55, 72, 5, 30, "Nordic_Region", osg::Vec3(62, 15, 0), {}, 0, 0}, // 北欧
    {78, 45, 55, 15, 30, "Central_Europe", osg::Vec3(50, 22, 0), {}, 0, 0}, // 中欧
    {79, 35, 45, 20, 30, "Balkans_Region", osg::Vec3(42, 25, 0), {}, 0, 0}, // 巴尔干
    {80, 25, 40, 35, 50, "Middle_East_Core", osg::Vec3(32, 42, 0), {}, 0, 0}, // 核心中东
    
    // 非洲主要区域
    {81, 20, 37, -20, 15, "North_Africa_Maghreb", osg::Vec3(30, -2, 0), {}, 0, 0},
    {82, 20, 37, 15, 40, "Northeast_Africa", osg::Vec3(28, 27, 0), {}, 0, 0},
    {83, 0, 20, -20, 20, "West_Central_Africa", osg::Vec3(10, 0, 0), {}, 0, 0},
    {84, -5, 15, 20, 50, "East_Africa_Region", osg::Vec3(5, 35, 0), {}, 0, 0},
    {85, -35, -5, 10, 35, "Southern_Africa_Region", osg::Vec3(-20, 22, 0), {}, 0, 0},
    
    // 东南亚各国
    {86, -10, 8, 95, 115, "Indonesia_Malaysia", osg::Vec3(-2, 105, 0), {}, 0, 0},
    {87, 0, 25, 95, 110, "Mainland_Southeast_Asia", osg::Vec3(15, 102, 0), {}, 0, 0}, // 泰国、越南
    {88, 5, 25, 110, 125, "Philippines_Region", osg::Vec3(15, 120, 0), {}, 0, 0},
    
    // 中亚和西亚
    {89, 30, 50, 50, 80, "Central_Asia_Region", osg::Vec3(40, 65, 0), {}, 0, 0},
    {90, 20, 40, 45, 65, "Southwest_Asia", osg::Vec3(30, 55, 0), {}, 0, 0},
    
    // 大洋洲补充区域
    {91, -50, -30, 165, 180, "New_Zealand_Region", osg::Vec3(-40, 175, 0), {}, 0, 0},
    {92, -20, 20, 140, 180, "Pacific_Islands_Region", osg::Vec3(0, 160, 0), {}, 0, 0}
};

} // namespace VIS4Earth

#endif // VIS4EARTH_GRAPH_VISER_GEOGRAPHIC_REGIONS_H 