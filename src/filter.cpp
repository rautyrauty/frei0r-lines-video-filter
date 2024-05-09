// ffmpeg -loglevel debug -i videoplayback.mp4 -vf "frei0r=straightlines:2000|10|n|y" -t 30 lovebff.mp4

#include <frei0r.hpp>
#include "FrameHandler.h"

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

frei0r::construct<LinesFilter> plugin("Straight lines",
	"Each frame is drawn only with random straight lines",
	"rautyrauty",
	3, 0,
	F0R_COLOR_MODEL_RGBA8888);
