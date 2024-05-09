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
