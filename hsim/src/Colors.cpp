//
//  Colors.cpp
//  hsim
//
//  Created by z on 2/8/19.
//

#include "hsim/Colors.hpp"

#include <array>
#include <cmath>

namespace hsim {

std::array<double, 3> Color2Lab(const Color &color)
{
  
  double r = color.Red() / 255.0;
  double g = color.Green() / 255.0;
  double b = color.Blue() / 255.0;
  double x, y, z;
  r = (r > 0.04045) ? pow((r + 0.055) / 1.055, 2.4) : r / 12.92;
  g = (g > 0.04045) ? pow((g + 0.055) / 1.055, 2.4) : g / 12.92;
  b = (b > 0.04045) ? pow((b + 0.055) / 1.055, 2.4) : b / 12.92;
  x = (r * 0.4124 + g * 0.3576 + b * 0.1805) / 0.95047;
  y = (r * 0.2126 + g * 0.7152 + b * 0.0722) / 1.00000;
  z = (r * 0.0193 + g * 0.1192 + b * 0.9505) / 1.08883;
  x = (x > 0.008856) ? pow(x, 1.0/3.0) : (7.787 * x) + 16.0/116.0;
  y = (y > 0.008856) ? pow(y, 1.0/3.0) : (7.787 * y) + 16.0/116.0;
  z = (z > 0.008856) ? pow(z, 1.0/3.0) : (7.787 * z) + 16.0/116.0;
  
  return std::array<double, 3>({(116 * y) - 16, 500 * (x - y), 200 * (y - z)});
}

// calculate the perceptual distance between colors in CIELAB
// https://github.com/THEjoezack/ColorMine/blob/master/ColorMine/ColorSpaces/Comparisons/Cie94Comparison.cs
/*
<= 1.0  Not perceptible by human eyes.
1 - 2  Perceptible through close observation.
2 - 10  Perceptible at a glance.
11 - 49  Colors are more similar than opposite
100  Colors are exact opposite
*/
double Color::Diff(const Color &lhs, const Color &rhs)
{
  std::array<double, 3> lhs_lab = Color2Lab(lhs);
  std::array<double, 3> rhs_lab = Color2Lab(rhs);

  double delta_l = lhs_lab[0] - rhs_lab[0];
  double delta_a = lhs_lab[1] - rhs_lab[1];
  double delta_b = lhs_lab[2] - rhs_lab[2];
  double c1 = sqrt(lhs_lab[1] * lhs_lab[1] + lhs_lab[2] * lhs_lab[2]);
  double c2 = sqrt(rhs_lab[1] * rhs_lab[1] + rhs_lab[2] * rhs_lab[2]);
  double delta_c = c1 - c2;
  double delta_h = delta_a * delta_a + delta_b * delta_b - delta_c * delta_c;
  delta_h = delta_h < 0 ? 0 : sqrt(delta_h);
  double sc = 1.0 + 0.045 * c1;
  double sh = 1.0 + 0.015 * c1;
  double delta_lklsl = delta_l / 1.0;
  double delta_ckcsc = delta_c / sc;
  double delta_hkhsh = delta_h / sh;
  return sqrt(delta_lklsl * delta_lklsl + delta_ckcsc * delta_ckcsc + delta_hkhsh * delta_hkhsh);
}



const std::vector<Color> Colors::all = {
  Color("Black", 0x000000),
  Color("Maroon", 0x800000),
  Color("Green", 0x008000),
  Color("Olive", 0x808000),
  Color("Navy", 0x000080),
  Color("Purple", 0x800080),
  Color("Teal", 0x008080),
  Color("Silver", 0xc0c0c0),
  Color("Grey", 0x808080),
  Color("Red", 0xff0000),
  Color("Lime", 0x00ff00),
  Color("Yellow", 0xffff00),
  Color("Blue", 0x0000ff),
  Color("Fuchsia", 0xff00ff),
  Color("Aqua", 0x00ffff),
  Color("White", 0xffffff),
  Color("Grey0", 0x000000),
  Color("NavyBlue", 0x00005f),
  Color("DarkBlue", 0x000087),
  Color("Blue3", 0x0000af),
  Color("Blue3", 0x0000d7),
  Color("Blue1", 0x0000ff),
  Color("DarkGreen", 0x005f00),
  Color("DeepSkyBlue4", 0x005f5f),
  Color("DeepSkyBlue4", 0x005f87),
  Color("DeepSkyBlue4", 0x005faf),
  Color("DodgerBlue3", 0x005fd7),
  Color("DodgerBlue2", 0x005fff),
  Color("Green4", 0x008700),
  Color("SpringGreen4", 0x00875f),
  Color("Turquoise4", 0x008787),
  Color("DeepSkyBlue3", 0x0087af),
  Color("DeepSkyBlue3", 0x0087d7),
  Color("DodgerBlue1", 0x0087ff),
  Color("Green3", 0x00af00),
  Color("SpringGreen3", 0x00af5f),
  Color("DarkCyan", 0x00af87),
  Color("LightSeaGreen", 0x00afaf),
  Color("DeepSkyBlue2", 0x00afd7),
  Color("DeepSkyBlue1", 0x00afff),
  Color("Green3", 0x00d700),
  Color("SpringGreen3", 0x00d75f),
  Color("SpringGreen2", 0x00d787),
  Color("Cyan3", 0x00d7af),
  Color("DarkTurquoise", 0x00d7d7),
  Color("Turquoise2", 0x00d7ff),
  Color("Green1", 0x00ff00),
  Color("SpringGreen2", 0x00ff5f),
  Color("SpringGreen1", 0x00ff87),
  Color("MediumSpringGreen", 0x00ffaf),
  Color("Cyan2", 0x00ffd7),
  Color("Cyan1", 0x00ffff),
  Color("DarkRed", 0x5f0000),
  Color("DeepPink4", 0x5f005f),
  Color("Purple4", 0x5f0087),
  Color("Purple4", 0x5f00af),
  Color("Purple3", 0x5f00d7),
  Color("BlueViolet", 0x5f00ff),
  Color("Orange4", 0x5f5f00),
  Color("Grey37", 0x5f5f5f),
  Color("MediumPurple4", 0x5f5f87),
  Color("SlateBlue3", 0x5f5faf),
  Color("SlateBlue3", 0x5f5fd7),
  Color("RoyalBlue1", 0x5f5fff),
  Color("Chartreuse4", 0x5f8700),
  Color("DarkSeaGreen4", 0x5f875f),
  Color("PaleTurquoise4", 0x5f8787),
  Color("SteelBlue", 0x5f87af),
  Color("SteelBlue3", 0x5f87d7),
  Color("CornflowerBlue", 0x5f87ff),
  Color("Chartreuse3", 0x5faf00),
  Color("DarkSeaGreen4", 0x5faf5f),
  Color("CadetBlue", 0x5faf87),
  Color("CadetBlue", 0x5fafaf),
  Color("SkyBlue3", 0x5fafd7),
  Color("SteelBlue1", 0x5fafff),
  Color("Chartreuse3", 0x5fd700),
  Color("PaleGreen3", 0x5fd75f),
  Color("SeaGreen3", 0x5fd787),
  Color("Aquamarine3", 0x5fd7af),
  Color("MediumTurquoise", 0x5fd7d7),
  Color("SteelBlue1", 0x5fd7ff),
  Color("Chartreuse2", 0x5fff00),
  Color("SeaGreen2", 0x5fff5f),
  Color("SeaGreen1", 0x5fff87),
  Color("SeaGreen1", 0x5fffaf),
  Color("Aquamarine1", 0x5fffd7),
  Color("DarkSlateGray2", 0x5fffff),
  Color("DarkRed", 0x870000),
  Color("DeepPink4", 0x87005f),
  Color("DarkMagenta", 0x870087),
  Color("DarkMagenta", 0x8700af),
  Color("DarkViolet", 0x8700d7),
  Color("Purple", 0x8700ff),
  Color("Orange4", 0x875f00),
  Color("LightPink4", 0x875f5f),
  Color("Plum4", 0x875f87),
  Color("MediumPurple3", 0x875faf),
  Color("MediumPurple3", 0x875fd7),
  Color("SlateBlue1", 0x875fff),
  Color("Yellow4", 0x878700),
  Color("Wheat4", 0x87875f),
  Color("Grey53", 0x878787),
  Color("LightSlateGrey", 0x8787af),
  Color("MediumPurple", 0x8787d7),
  Color("LightSlateBlue", 0x8787ff),
  Color("Yellow4", 0x87af00),
  Color("DarkOliveGreen3", 0x87af5f),
  Color("DarkSeaGreen", 0x87af87),
  Color("LightSkyBlue3", 0x87afaf),
  Color("LightSkyBlue3", 0x87afd7),
  Color("SkyBlue2", 0x87afff),
  Color("Chartreuse2", 0x87d700),
  Color("DarkOliveGreen3", 0x87d75f),
  Color("PaleGreen3", 0x87d787),
  Color("DarkSeaGreen3", 0x87d7af),
  Color("DarkSlateGray3", 0x87d7d7),
  Color("SkyBlue1", 0x87d7ff),
  Color("Chartreuse1", 0x87ff00),
  Color("LightGreen", 0x87ff5f),
  Color("LightGreen", 0x87ff87),
  Color("PaleGreen1", 0x87ffaf),
  Color("Aquamarine1", 0x87ffd7),
  Color("DarkSlateGray1", 0x87ffff),
  Color("Red3", 0xaf0000),
  Color("DeepPink4", 0xaf005f),
  Color("MediumVioletRed", 0xaf0087),
  Color("Magenta3", 0xaf00af),
  Color("DarkViolet", 0xaf00d7),
  Color("Purple", 0xaf00ff),
  Color("DarkOrange3", 0xaf5f00),
  Color("IndianRed", 0xaf5f5f),
  Color("HotPink3", 0xaf5f87),
  Color("MediumOrchid3", 0xaf5faf),
  Color("MediumOrchid", 0xaf5fd7),
  Color("MediumPurple2", 0xaf5fff),
  Color("DarkGoldenrod", 0xaf8700),
  Color("LightSalmon3", 0xaf875f),
  Color("RosyBrown", 0xaf8787),
  Color("Grey63", 0xaf87af),
  Color("MediumPurple2", 0xaf87d7),
  Color("MediumPurple1", 0xaf87ff),
  Color("Gold3", 0xafaf00),
  Color("DarkKhaki", 0xafaf5f),
  Color("NavajoWhite3", 0xafaf87),
  Color("Grey69", 0xafafaf),
  Color("LightSteelBlue3", 0xafafd7),
  Color("LightSteelBlue", 0xafafff),
  Color("Yellow3", 0xafd700),
  Color("DarkOliveGreen3", 0xafd75f),
  Color("DarkSeaGreen3", 0xafd787),
  Color("DarkSeaGreen2", 0xafd7af),
  Color("LightCyan3", 0xafd7d7),
  Color("LightSkyBlue1", 0xafd7ff),
  Color("GreenYellow", 0xafff00),
  Color("DarkOliveGreen2", 0xafff5f),
  Color("PaleGreen1", 0xafff87),
  Color("DarkSeaGreen2", 0xafffaf),
  Color("DarkSeaGreen1", 0xafffd7),
  Color("PaleTurquoise1", 0xafffff),
  Color("Red3", 0xd70000),
  Color("DeepPink3", 0xd7005f),
  Color("DeepPink3", 0xd70087),
  Color("Magenta3", 0xd700af),
  Color("Magenta3", 0xd700d7),
  Color("Magenta2", 0xd700ff),
  Color("DarkOrange3", 0xd75f00),
  Color("IndianRed", 0xd75f5f),
  Color("HotPink3", 0xd75f87),
  Color("HotPink2", 0xd75faf),
  Color("Orchid", 0xd75fd7),
  Color("MediumOrchid1", 0xd75fff),
  Color("Orange3", 0xd78700),
  Color("LightSalmon3", 0xd7875f),
  Color("LightPink3", 0xd78787),
  Color("Pink3", 0xd787af),
  Color("Plum3", 0xd787d7),
  Color("Violet", 0xd787ff),
  Color("Gold3", 0xd7af00),
  Color("LightGoldenrod3", 0xd7af5f),
  Color("Tan", 0xd7af87),
  Color("MistyRose3", 0xd7afaf),
  Color("Thistle3", 0xd7afd7),
  Color("Plum2", 0xd7afff),
  Color("Yellow3", 0xd7d700),
  Color("Khaki3", 0xd7d75f),
  Color("LightGoldenrod2", 0xd7d787),
  Color("LightYellow3", 0xd7d7af),
  Color("Grey84", 0xd7d7d7),
  Color("LightSteelBlue1", 0xd7d7ff),
  Color("Yellow2", 0xd7ff00),
  Color("DarkOliveGreen1", 0xd7ff5f),
  Color("DarkOliveGreen1", 0xd7ff87),
  Color("DarkSeaGreen1", 0xd7ffaf),
  Color("Honeydew2", 0xd7ffd7),
  Color("LightCyan1", 0xd7ffff),
  Color("Red1", 0xff0000),
  Color("DeepPink2", 0xff005f),
  Color("DeepPink1", 0xff0087),
  Color("DeepPink1", 0xff00af),
  Color("Magenta2", 0xff00d7),
  Color("Magenta1", 0xff00ff),
  Color("OrangeRed1", 0xff5f00),
  Color("IndianRed1", 0xff5f5f),
  Color("IndianRed1", 0xff5f87),
  Color("HotPink", 0xff5faf),
  Color("HotPink", 0xff5fd7),
  Color("MediumOrchid1", 0xff5fff),
  Color("DarkOrange", 0xff8700),
  Color("Salmon1", 0xff875f),
  Color("LightCoral", 0xff8787),
  Color("PaleVioletRed1", 0xff87af),
  Color("Orchid2", 0xff87d7),
  Color("Orchid1", 0xff87ff),
  Color("Orange1", 0xffaf00),
  Color("SandyBrown", 0xffaf5f),
  Color("LightSalmon1", 0xffaf87),
  Color("LightPink1", 0xffafaf),
  Color("Pink1", 0xffafd7),
  Color("Plum1", 0xffafff),
  Color("Gold1", 0xffd700),
  Color("LightGoldenrod2", 0xffd75f),
  Color("LightGoldenrod2", 0xffd787),
  Color("NavajoWhite1", 0xffd7af),
  Color("MistyRose1", 0xffd7d7),
  Color("Thistle1", 0xffd7ff),
  Color("Yellow1", 0xffff00),
  Color("LightGoldenrod1", 0xffff5f),
  Color("Khaki1", 0xffff87),
  Color("Wheat1", 0xffffaf),
  Color("Cornsilk1", 0xffffd7),
  Color("Grey100", 0xffffff),
  Color("Grey3", 0x080808),
  Color("Grey7", 0x121212),
  Color("Grey11", 0x1c1c1c),
  Color("Grey15", 0x262626),
  Color("Grey19", 0x303030),
  Color("Grey23", 0x3a3a3a),
  Color("Grey27", 0x444444),
  Color("Grey30", 0x4e4e4e),
  Color("Grey35", 0x585858),
  Color("Grey39", 0x626262),
  Color("Grey42", 0x6c6c6c),
  Color("Grey46", 0x767676),
  Color("Grey50", 0x808080),
  Color("Grey54", 0x8a8a8a),
  Color("Grey58", 0x949494),
  Color("Grey62", 0x9e9e9e),
  Color("Grey66", 0xa8a8a8),
  Color("Grey70", 0xb2b2b2),
  Color("Grey74", 0xbcbcbc),
  Color("Grey78", 0xc6c6c6),
  Color("Grey82", 0xd0d0d0),
  Color("Grey85", 0xdadada),
  Color("Grey89", 0xe4e4e4),
  Color("Grey93", 0xeeeeee)
};

}
