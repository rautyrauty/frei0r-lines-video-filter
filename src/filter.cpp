// ffmpeg -loglevel debug -i videoplayback.mp4 -vf "frei0r=straightlines:2000|10|n|y" -t 30 lovebff.mp4

#include <frei0r.hpp>
#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>
#include <random>
#include <memory>
#include <cfloat>

struct Pos
{
	int32_t x;
	int32_t y;
};

class FrameHandler
{
public:
	FrameHandler(const uint32_t width, const uint32_t height, const double num_of_lines, const double sample_size, const bool should_draw_extreme_lines, const bool long_line_mode);

	void FrameProcessing(uint32_t* out, const uint32_t* in);

private:
	void FindEdges(int32_t& x0, int32_t& y0, int32_t& x1, int32_t& y1) const;

	uint32_t GetPixelValue(int32_t x, int32_t y, const uint32_t* in) const;
	double GetLineValue(int32_t x0, int32_t y0, int32_t x1, int32_t y1,const uint32_t* in, const uint32_t* out) const;

	void UpdatePoints(const uint32_t* in);

	void DrawPixel(int32_t x, int32_t y, float brightess, uint32_t* out) const;
	void DrawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, float brightness, uint32_t* out) const;

	const uint32_t m_kNumOfLines;
	const uint32_t m_kSampleSize;
	const bool m_kShouldDrawExtremePoints;
	const bool m_kLongLineMode;

	const uint32_t m_kWidth;
	const uint32_t m_kHeight;

	std::mt19937 rng;

	std::vector<Pos> m_darkest_points;
	std::vector<std::vector<Pos>> m_extreme_points;
};

// The main class in frei0r. It is responsible for user settings and
// the update function, which takes the initial frame and outputs the result
class LinesFilter : public frei0r::filter
{
public:
	LinesFilter(unsigned int, unsigned int);
	void update(double time, uint32_t* out, const uint32_t* in) override;

private:
	std::unique_ptr<FrameHandler> m_frame_handler;

	// User settings:
	double m_num_of_lines;
	double m_sample_size;
	bool m_should_draw_extreme_lines;
	bool m_long_line_mode;
};

LinesFilter::LinesFilter(unsigned int, unsigned int) :
	m_frame_handler(nullptr),
	m_num_of_lines(1000),
	m_sample_size(10),
	m_should_draw_extreme_lines(false),
	m_long_line_mode(true)
{
	register_param(m_num_of_lines, "Number of Lines", "Default: 1000");
	register_param(m_sample_size, "Sample size", "Number of attempts to build a line. Default: 10");
	register_param(m_should_draw_extreme_lines, "Extreme Line Mode", "Should draw the extreme lines? Default: false");
	register_param(m_long_line_mode, "Long Line Mode", "Should draw lines to the end of the screen? Default: true");

	srand(time(0));
}

void LinesFilter::update(double, uint32_t* out, const uint32_t* in)
{
	/*
	 * Why didn't I initialize m_frame_handler in the constructor?
	 * Only in this function do we get the parameter values.
	 * This is one of the main reasons why this class exists at all.
	 */
	if (not m_frame_handler) m_frame_handler = std::make_unique<FrameHandler>(width, height, m_num_of_lines, m_sample_size, m_should_draw_extreme_lines, m_long_line_mode);

	std::fill(out, out + size, static_cast<char>(255)); // making the output frame white

	m_frame_handler->FrameProcessing(out, in);
}

FrameHandler::FrameHandler(const uint32_t width, const uint32_t height, const double num_of_lines, const double sample_size, const bool should_draw_extreme_lines, const bool long_line_mode) :
	m_kNumOfLines(num_of_lines),
	m_kSampleSize(sample_size),
	m_kShouldDrawExtremePoints(should_draw_extreme_lines),
	m_kLongLineMode(long_line_mode),
	m_kWidth(width),
	m_kHeight(height),
	rng(std::chrono::steady_clock::now().time_since_epoch().count())
{
	m_darkest_points.reserve(m_kWidth * m_kHeight);
}
void FrameHandler::FrameProcessing(uint32_t* out, const uint32_t* in)
{
	UpdatePoints(in);

	int32_t x0, y0, x1, y1;

	if (m_kShouldDrawExtremePoints)
	{
		for (int32_t h = 0; h < int(m_extreme_points.size()) - 1; h += 1)
		{
			for (uint32_t w = 0; ((w < m_extreme_points[h].size()) and (w < m_extreme_points[h + 1].size())); w += 1)
			{
				x0 = m_extreme_points[  h  ][w].x; y0 = m_extreme_points[  h  ][w].y;
				x1 = m_extreme_points[h + 1][w].x; y1 = m_extreme_points[h + 1][w].y;

				if (abs(x0 - x1) + abs(y0 - y1) < 15)
				{
					FindEdges(x0, y0, x1, y1);
					DrawLine(x0, y0, x1, y1, 0.25, out);
				}
			}
		}
	}

	std::uniform_real_distribution<double> urd{ 0.0, 180.0 };
	double random_angle;
	double current_darkest_value;
	Pos Darkest1, Darkest2;
	Pos Tmp;

	for (uint32_t i = 0; i < m_kNumOfLines; i += 1)
	{
		if (m_darkest_points.empty()) return;

		current_darkest_value = DBL_MAX;
		Darkest1 = { 0,0 };
		Darkest2 = { 0,0 };

		Tmp = m_darkest_points.back();
		m_darkest_points.pop_back();

		for (uint32_t j = 0; j < m_kSampleSize; j += 1)
		{
			random_angle = urd(rng);
			x0 = Tmp.x;
			y0 = Tmp.y;
			x1 = x0 + (int)(200.0 * cos(random_angle * 3.14 / 180.));
			y1 = y0 + (int)(200.0 * sin(random_angle * 3.14 / 180.));

			if (m_kLongLineMode) FindEdges(x0, y0, x1, y1);
			double lnbrh = GetLineValue(x0, y0, x1, y1, in,out);
			if (lnbrh < current_darkest_value)
			{
				current_darkest_value = lnbrh;
				Darkest1.x = x0;
				Darkest1.y = y0;
				Darkest2.x = x1;
				Darkest2.y = y1;
			}
		}
		DrawLine(Darkest1.x, Darkest1.y, Darkest2.x, Darkest2.y, 0.125, out);
	}
}

void FrameHandler::FindEdges(int32_t& x0, int32_t& y0, int32_t& x1, int32_t& y1) const
{
	if ((x0 == x1) and (y0 == y1)) return;

	if (x0 == x1)
	{
		y0 = 0;
		y1 = m_kHeight - 1;
		return;
	}

	if (y0 == y1)
	{
		x0 = 0;
		x1 = m_kWidth - 1;
		return;
	}

	bool flag = false;

	double tgA = double(y1 - y0)/double(x1 - x0);

	int32_t y2 = 0;
	int32_t x2 = double(y2 - y0) / tgA + x0;
	if ((x2 >= 0) and (x2 < m_kWidth))
	{
		x0 = x2;
		y0 = y2;
		flag = true;
	}
	y2 = m_kHeight - 1;
	x2 = double(y2 - y0) / tgA + x0;
	if ((x2 >= 0) and (x2 < m_kWidth))
	{
		if (flag)
		{
			x1 = x2;
			y1 = y2;
			return;
		}
		else
		{
			x0 = x2;
			y0 = y2;
			flag = true;
		}
	}
	x2 = 0;
	y2 = tgA * double(x2 - x0) + y0;
	if ((y2 >= 0) and (y2 < m_kHeight))
	{
		if (flag)
		{
			x1 = x2;
			y1 = y2;
			return;
		}
		else
		{
			x0 = x2;
			y0 = y2;
			flag = true;
		}
	}
	x2 = m_kWidth - 1;
	y2 = tgA * double(x2 - x0) + y0;
	if ((y2 >= 0) and (y2 < m_kHeight))
	{
		x1 = x2;
		y1 = y2;
		return;
	}
}

uint32_t FrameHandler::GetPixelValue(int32_t x, int32_t y, const uint32_t* in) const
{
	return (*(reinterpret_cast<const uint8_t*>(in + x + y * m_kWidth) + 0)   //r
		  + *(reinterpret_cast<const uint8_t*>(in + x + y * m_kWidth) + 1)   //g
		  + *(reinterpret_cast<const uint8_t*>(in + x + y * m_kWidth) + 2)); //b
}

double FrameHandler::GetLineValue(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const uint32_t* in, const uint32_t* out) const
{
	/*
	 * https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	 */
	int32_t line_brightness = 0;
	uint32_t counter = 0;

	const int deltaX = abs(x2 - x1);
	const int deltaY = abs(y2 - y1);
	const int signX = x1 < x2 ? 1 : -1;
	const int signY = y1 < y2 ? 1 : -1;
	int error = deltaX - deltaY;
	if ((x2 >= 0) and (y2 >= 0) and (x2 < m_kWidth) and (y2 < m_kHeight))
	{
		line_brightness += GetPixelValue(x2, y2, in);
		line_brightness -= GetPixelValue(x2, y2, out);
		counter += 1;
	}
	while (((x1 != x2) or (y1 != y2)) and (x1 >= 0) and (y1 >= 0) and (x1 < m_kWidth) and (y1 < m_kHeight))
	{
		line_brightness += GetPixelValue(x1, y1, in);
		line_brightness -= GetPixelValue(x1, y1, out);
		counter += 1;
		int error2 = error * 2;
		if (error2 > -deltaY)
		{
			error -= deltaY;
			x1 += signX;
		}
		if (error2 < deltaX)
		{
			error += deltaX;
			y1 += signY;
		}
	}
	return static_cast<double>(line_brightness) / static_cast<double>(counter);
}

void FrameHandler::UpdatePoints(const uint32_t* in)
{
	m_darkest_points.clear();
	for (int32_t w = 1; w < m_kWidth - 1; w += 1)
	{
		for (int32_t h = 1; h < m_kHeight - 1; h += 1)
		{
			if (GetPixelValue(w, h, in) <= 300)
			{
				m_darkest_points.push_back(Pos{ w,h });
			}
		}
	}
	std::shuffle(m_darkest_points.begin(), m_darkest_points.end(), rng);


	if (m_kShouldDrawExtremePoints)
	{
		m_extreme_points.clear();
		std::vector<Pos> tmp;
		tmp.reserve(20);
		for (int32_t j = 1; j < m_kHeight - 1; j += 1)
		{
			tmp.clear();
			for (int32_t i = 1; i < m_kWidth - 1; i += 1)
			{
				if (GetPixelValue(i, j, in) < 300)
				{
					int result = 0;
					if (GetPixelValue(i + 1, j, in) > 300) result += 1;
					if (GetPixelValue(i - 1, j, in) > 300) result += 1;
					if (GetPixelValue(i, j + 1, in) > 300) result += 1;
					if (GetPixelValue(i, j - 1, in) > 300) result += 1;
					if (result > 1) tmp.push_back(Pos{ i,j });
				}
			}
			if (not tmp.empty()) m_extreme_points.push_back(tmp);
		}
	}
}

void FrameHandler::DrawPixel(int32_t x, int32_t y, float brightness, uint32_t* out) const
{
	if (y >= m_kHeight) return;
	if (x >= m_kWidth) return;

	uint8_t v = 255 * brightness;
	if (*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 0) > v)
	{
		*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 0) -= v;
		*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 1) -= v;
		*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 2) -= v;
	}
	else
	{
		*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 0) = 0;
		*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 1) = 0;
		*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 2) = 0;
	}
	*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 3) = 255;
}

void FrameHandler::DrawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1 , float brightness, uint32_t* out) const
{
	 // https://en.wikipedia.org/wiki/Xiaolin_Wu%27s_line_algorithm

	auto ipart = [](float x) -> int {return int(std::floor(x)); };
	auto round = [](float x) -> float {return std::round(x); };
	auto fpart = [](float x) -> float {return x - std::floor(x); };
	auto rfpart = [=](float x) -> float {return 1 - fpart(x); };

	const bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		std::swap(x0, y0);
		std::swap(x1, y1);
	}
	if (x0 > x1) {
		std::swap(x0, x1);
		std::swap(y0, y1);
	}

	const float dx = x1 - x0;
	const float dy = y1 - y0;
	const float gradient = (dx == 0) ? 1 : dy / dx;

	int xpx11;
	float intery;
	{
		const float xend = round(x0);
		const float yend = y0 + gradient * (xend - x0);
		const float xgap = rfpart(x0 + 0.5);
		xpx11 = int(xend);
		const int ypx11 = ipart(yend);
		if (steep) {
			DrawPixel(ypx11, xpx11, rfpart(yend) * xgap *brightness, out);
			DrawPixel(ypx11 + 1, xpx11, fpart(yend) * xgap * brightness, out);
		}
		else {
			DrawPixel(xpx11, ypx11, rfpart(yend) * xgap * brightness, out);
			DrawPixel(xpx11, ypx11 + 1, fpart(yend) * xgap * brightness, out);
		}
		intery = yend + gradient;
	}

	int xpx12;
	{
		const float xend = round(x1);
		const float yend = y1 + gradient * (xend - x1);
		const float xgap = rfpart(x1 + 0.5);
		xpx12 = int(xend);
		const int ypx12 = ipart(yend);
		if (steep) {
			DrawPixel(ypx12, xpx12, rfpart(yend) * xgap * brightness, out);
			DrawPixel(ypx12 + 1, xpx12, fpart(yend) * xgap * brightness, out);
		}
		else {
			DrawPixel(xpx12, ypx12, rfpart(yend) * xgap * brightness, out);
			DrawPixel(xpx12, ypx12 + 1, fpart(yend) * xgap * brightness, out);
		}
	}

	if (steep) {
		for (int x = xpx11 + 1; x < xpx12; x++) {
			DrawPixel(ipart(intery), x, rfpart(intery) * brightness, out);
			DrawPixel(ipart(intery) + 1, x, fpart(intery) * brightness, out);
			intery += gradient;
		}
	}
	else {
		for (int x = xpx11 + 1; x < xpx12; x++) {
			DrawPixel(x, ipart(intery), rfpart(intery) * brightness, out);
			DrawPixel(x, ipart(intery) + 1, fpart(intery) * brightness, out);
			intery += gradient;
		}
	}
}


frei0r::construct<LinesFilter> plugin("Straight lines",
	"Each frame is drawn only with random straight lines",
	"rautyrauty",
	3, 0,
	F0R_COLOR_MODEL_RGBA8888);
