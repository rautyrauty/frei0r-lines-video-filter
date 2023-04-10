//ffmpeg -loglevel debug -i videoplayback.mp4 -vf "frei0r=FrameHandler" -t 10 of.mp4

#include <frei0r.hpp>
#include <iostream>

/* 
 * Каждый кадр будет мысленно поделен на квадратные поля (просто на квадраты) со стороной m_side_size.
 * Это сделано для того, чтобы ускорить и упростить нахождение средней яркости пикселей,
 * попавших под линию. Это значение будет сравниваться с m_max_brightness, решая - отрисовывать
 * данную линию или нет. 
 * Сочувствую тем, кто захочет понять че тут происходит.
 */

// Этот класс отвечает только за параметры вводимые пользователем и функцию update 
class LinesFilter : public frei0r::filter
{
public:
	LinesFilter(unsigned int, unsigned int);
	~LinesFilter() override;

	// Главная ф-ция отвечающая за обработку кадров 
	void update(double time, uint32_t* out, const uint32_t* in) override;

private:

	//параметры:

	double m_side_size;

	//Класс отвечающий почти что за всё
	class FrameHandler
	{
	public:
		FrameHandler(const uint32_t width, const uint32_t height, const double side_size);
		~FrameHandler();

		void FrameProcessing(uint32_t* out, const uint32_t* in);

	private:
		void UpdateSquaresBrightness(const uint32_t* in);
		void GenerateExtremePoints();
		bool IsLineFitsBrightness(int32_t x0, int32_t y0, int32_t x1, int32_t y1) const;
		void DrawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t* out, const uint32_t* in) const;
		void plot(int32_t x, int32_t y, float brightess, uint32_t* out, const uint32_t* in) const;

		//Целая часть параметра m_side_size
		const uint32_t m_kRoundSideSize;

		const uint32_t m_kWidth;
		const uint32_t m_kHeight;

		//количество квадратов по ширине
		const uint32_t m_kSquaresWidthCount;
		//количество квадратов по длине
		const uint32_t m_kSquaresHeightCount;

		// Двумерный массив размерности m_kSquaresWidthCount+1 * m_kSquaresHeightCount+1 
		// где хранятся средняя яркость пикселей находящихся в квадратах.
		//Высчитывается отдельно для каждого кадра в UpdateSquaresBrightness(const uint32_t* in)
		uint32_t** m_SquaresBrightness;

		//Максимальная средняя яркость пикселей попадающих под линию.
		//Высчитывается отдельно для каждого кадра в UpdateSquaresBrightness(const uint32_t* in)
		uint32_t m_max_line_brightness;

		enum class VideoSide
		{
			UP    = 0,
			DOWN  = 1,
			LEFT  = 2,
			RIGHT = 3
		};

		// Обёртка для двумерного массива размерности 4 * (m_kSquaresHeightCount или m_kSquaresWidthCount)
		// где хранятся крайние точки потенциальных линий. [VideoSide][Номер квадрата сверху-вниз или слева-направо]
		class ExtremePoints 
		{
		public:
			uint32_t*& operator[](VideoSide s) { return arr[static_cast<int>(VideoSide::UP)]; }
		private:
			uint32_t* arr[4];
		} m_ExtremePoints;
	}*m_frame_handler;
};

LinesFilter::LinesFilter(unsigned int, unsigned int) :
	m_frame_handler(nullptr),
	//Значения по умолчанию:
	m_side_size(15)
{
	register_param(m_side_size, "SideSize",
		"Side size of square fields(didn't ask). This parameter is responsible for lines density. Min: 30");

	//srand(time(0));
}

LinesFilter::~LinesFilter()
{
	delete m_frame_handler;
}

void LinesFilter::update(double, uint32_t* out, const uint32_t* in)
{
	/*
	 * Спросите почему я не инициализировал m_frame_handler в конструкторе ?
	 * Ответ: только здесь мы узнаем значения параметров m_side_size и m_max_brightness.
	 * Это одна из главных причин почему этот класс впринципе существует
	 */
	if (not m_frame_handler) m_frame_handler = new FrameHandler(width, height, m_side_size);

	std::fill(out, out + size, static_cast<char>(255)); // делаем кадр белым

	m_frame_handler->FrameProcessing(out, in);
}


LinesFilter::FrameHandler::FrameHandler(const uint32_t width, const uint32_t height, const double side_size) :
	m_kRoundSideSize(side_size >= 5 ? static_cast<uint32_t>(side_size) : 5),
	m_kWidth(width),
	m_kHeight(height),
	m_kSquaresWidthCount(m_kWidth  / m_kRoundSideSize),
	m_kSquaresHeightCount(m_kHeight / m_kRoundSideSize),
	m_max_line_brightness(0)
{
	m_SquaresBrightness = new uint32_t * [m_kSquaresWidthCount];
	for (uint32_t i = 0; i < m_kSquaresWidthCount; i += 1)
	{
		m_SquaresBrightness[i] = new uint32_t[m_kSquaresHeightCount];
	}

	//Можно было бы сделать массив статичным, но мне так больше нравится
	m_ExtremePoints[VideoSide::UP]    = new uint32_t[m_kSquaresWidthCount];
	m_ExtremePoints[VideoSide::DOWN]  = new uint32_t[m_kSquaresWidthCount];
	m_ExtremePoints[VideoSide::LEFT]  = new uint32_t[m_kSquaresHeightCount];
	m_ExtremePoints[VideoSide::RIGHT] = new uint32_t[m_kSquaresHeightCount];
}

LinesFilter::FrameHandler::~FrameHandler()
{
	delete[] m_ExtremePoints[VideoSide::UP];
	delete[] m_ExtremePoints[VideoSide::DOWN];
	delete[] m_ExtremePoints[VideoSide::LEFT];
	delete[] m_ExtremePoints[VideoSide::RIGHT];

	for (uint32_t i = 0; i <= m_kSquaresWidthCount; i += 1)
	{
		delete[] m_SquaresBrightness[i];
	}
	delete[] m_SquaresBrightness;
}

void LinesFilter::FrameHandler::FrameProcessing(uint32_t* out, const uint32_t* in)
{
	UpdateSquaresBrightness(in);

	GenerateExtremePoints();

	for (uint32_t i = 0; i < m_kSquaresWidthCount ; i += 1)
	{
		for (uint32_t j = 0; j < m_kSquaresWidthCount; j += 1)
		{
			if (IsLineFitsBrightness(m_ExtremePoints[VideoSide::UP][i], 0, m_ExtremePoints[VideoSide::DOWN][j], m_kHeight-1 - m_kHeight % m_kRoundSideSize))
			{
				DrawLine(m_ExtremePoints[VideoSide::UP][i], 0, m_ExtremePoints[VideoSide::DOWN][j], m_kHeight - 1 - m_kHeight%m_kRoundSideSize, out, in);
			}
		}
	}
	for (uint32_t i = 0; i < m_kSquaresHeightCount; i += 1)
	{
		for (uint32_t j = 0; j < m_kSquaresHeightCount; j += 1)
		{
			if (IsLineFitsBrightness(0, m_ExtremePoints[VideoSide::LEFT][i], m_kWidth - 1 - m_kWidth % m_kRoundSideSize, m_ExtremePoints[VideoSide::RIGHT][j]))
			{
				DrawLine(0, m_ExtremePoints[VideoSide::LEFT][i], m_kWidth - 1 - m_kWidth % m_kRoundSideSize, m_ExtremePoints[VideoSide::RIGHT][j], out, in);
			}
		}
	}

	/*for (uint32_t i = 2; i < m_kSquaresWidthCount-2; i += 1)
	{
		for (uint32_t j = 2; j < m_kSquaresHeightCount-2; j += 1)
		{
			if (IsLineFitsBrightness(m_ExtremePoints[VideoSide::UP][i], 0, 0, m_ExtremePoints[VideoSide::LEFT][j]))
			{
				DrawLine(m_ExtremePoints[VideoSide::UP][i], 0, 0, m_ExtremePoints[VideoSide::LEFT][j], out);
			}
			if (IsLineFitsBrightness(m_ExtremePoints[VideoSide::UP][i], 0, m_kWidth-1 - m_kWidth % m_kRoundSideSize, m_ExtremePoints[VideoSide::RIGHT][j]))
			{
				DrawLine(m_ExtremePoints[VideoSide::UP][i], 0, m_kWidth - 1 - m_kWidth % m_kRoundSideSize, m_ExtremePoints[VideoSide::RIGHT][j], out);
			}
			if (IsLineFitsBrightness(m_ExtremePoints[VideoSide::DOWN][i], m_kHeight-1 - m_kHeight % m_kRoundSideSize, 0, m_ExtremePoints[VideoSide::LEFT][j]))
			{
				DrawLine(m_ExtremePoints[VideoSide::DOWN][i], m_kHeight - 1 - m_kHeight % m_kRoundSideSize, 0, m_ExtremePoints[VideoSide::LEFT][j], out);
			}
			if (IsLineFitsBrightness(m_ExtremePoints[VideoSide::DOWN][i], m_kHeight - 1 - m_kHeight % m_kRoundSideSize, m_kWidth-1 - m_kWidth % m_kRoundSideSize, m_ExtremePoints[VideoSide::RIGHT][j]))
			{
				DrawLine(m_ExtremePoints[VideoSide::DOWN][i], m_kHeight - 1 - m_kHeight % m_kRoundSideSize, m_kWidth - 1 - m_kWidth % m_kRoundSideSize, m_ExtremePoints[VideoSide::RIGHT][j], out);
			}
		}
	}*/
}

void LinesFilter::FrameHandler::UpdateSquaresBrightness(const uint32_t* in)
{
	for (uint32_t i = 0; i < m_kSquaresWidthCount; i += 1)
	{
		std::fill(m_SquaresBrightness[i], m_SquaresBrightness[i]+m_kSquaresHeightCount, '\0');
	}
	uint32_t current_brightness = 0;
	for (uint32_t w = 0; w < m_kWidth - m_kWidth% m_kRoundSideSize; w += 1)
	{
		for (uint32_t h = 0; h < m_kHeight - m_kHeight % m_kRoundSideSize; h += 1)
		{
			// Формула для нахождения средней яркости
			current_brightness = static_cast<uint32_t>
				 (0.2126 * static_cast<double>(*(reinterpret_cast<const uint8_t*>(in + w + h * m_kWidth) + 0))   //r
				+ 0.7152 * static_cast<double>(*(reinterpret_cast<const uint8_t*>(in + w + h * m_kWidth) + 1))   //g
				+ 0.0722 * static_cast<double>(*(reinterpret_cast<const uint8_t*>(in + w + h * m_kWidth) + 2))); //b
			m_SquaresBrightness[w / m_kRoundSideSize][h / m_kRoundSideSize] += current_brightness;
			m_max_line_brightness += current_brightness;
		}

	}

	for (uint32_t w = 0; w < m_kSquaresWidthCount; w += 1)
	{
		for (uint32_t h = 0; h < m_kSquaresHeightCount; h += 1)
		{
			//привожу к среднему значению
			m_SquaresBrightness[w][h] /= (m_kRoundSideSize * m_kRoundSideSize);
		}
	}

	m_max_line_brightness /= (m_kHeight * m_kWidth) * 5;
	//std::cout << m_max_line_brightness << '\n';
}

void LinesFilter::FrameHandler::GenerateExtremePoints()
{
	for (uint32_t i = 0; i < m_kSquaresWidthCount; i += 1)
	{
		m_ExtremePoints[VideoSide::UP]  [i] = rand() % (m_kRoundSideSize - 2) + 1 + i*m_kRoundSideSize;
		m_ExtremePoints[VideoSide::DOWN][i] = rand() % (m_kRoundSideSize - 2) + 1 + i * m_kRoundSideSize;
	}
	for (uint32_t i = 0; i < m_kSquaresHeightCount; i += 1)
	{
		m_ExtremePoints[VideoSide::LEFT] [i] = rand() % (m_kRoundSideSize - 2) + 1 + i * m_kRoundSideSize;
		m_ExtremePoints[VideoSide::RIGHT][i] = rand() % (m_kRoundSideSize - 2) + 1 + i * m_kRoundSideSize;
	}
}

bool LinesFilter::FrameHandler::IsLineFitsBrightness(int32_t x0, int32_t y0, int32_t x1, int32_t y1) const
{
	/*
	 * Выяснять какие значения яркости квадратных полей будут учитываться при вычислении
	 * средней яркости пикселей, попавших под линию, будет алгоритм Брезенхема (не совсем, но идея взята оттуда).
	 * https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	 * Квадрат, по середине которого проходит линия, будет учитываться два раза, но это не баг - это фича!
	 * 
	 */


	uint32_t avg_line_brightness = 0;
	uint32_t square_counter = 0;

	if (y1 / m_kRoundSideSize == y0 / m_kRoundSideSize)
	{
		uint32_t i = x0 / m_kRoundSideSize;
		for (; i < x1 / m_kRoundSideSize; i += 1) avg_line_brightness += m_SquaresBrightness[i][y0/m_kRoundSideSize];
		avg_line_brightness /= (i - x0 / m_kRoundSideSize);
		return (avg_line_brightness <= m_max_line_brightness);
	}
	if (x1 / m_kRoundSideSize == x0 / m_kRoundSideSize)
	{
		uint32_t i = y0 / m_kRoundSideSize;
		for (; i < y1 / m_kRoundSideSize; i += 1) avg_line_brightness += m_SquaresBrightness[x0 / m_kRoundSideSize][i];
		avg_line_brightness /= (i - y0 / m_kRoundSideSize);
		return (avg_line_brightness <= m_max_line_brightness);
	}
	if (y0 > y1)
	{
		int32_t tmp = x0;
		x0 = x1;
		x1 = tmp;
		tmp = y0;
		y0 = y1;
		y1 = tmp;
	}
	
	int32_t dx = abs(x1 - x0);
	int32_t dy = y1 - y0;
	bool zerkalim = (dy > dx);
	if (zerkalim)
	{
		int32_t tmp = x0;
		x0 = y0;
		y0 = tmp;
		tmp = x1;
		x1 = y1;
		y1 = tmp;
	}
	double tgA = double(y1 - y0) / double(x1 - x0);

	int32_t direction_x = (x1 - x0 > 0 ? 1 : -1);


	int32_t tmp = x0 / m_kRoundSideSize * m_kRoundSideSize + m_kRoundSideSize / 2;
	if ((direction_x == 1) and (tmp < x0))
	{
		tmp += m_kRoundSideSize;
	}
	else if ((direction_x == -1) and (tmp > x0))
	{
		tmp -= m_kRoundSideSize;
	}
	y0 += (tmp - x0) * tgA;
	x0 = tmp;
	double error = 0;
	int y = y0 / m_kRoundSideSize;

	if (y1 / m_kRoundSideSize == y0 / m_kRoundSideSize)
	{
		uint32_t i = x0 / m_kRoundSideSize;
		for (; i < x1 / m_kRoundSideSize; i += 1) avg_line_brightness += m_SquaresBrightness[i][y0 / m_kRoundSideSize];
		avg_line_brightness /= (i - x0 / m_kRoundSideSize);
		return (avg_line_brightness <= m_max_line_brightness);
	}
	if (x1 / m_kRoundSideSize == x0 / m_kRoundSideSize)
	{
		uint32_t i = y0 / m_kRoundSideSize;
		for (; i < y1 / m_kRoundSideSize; i += 1) avg_line_brightness += m_SquaresBrightness[x0 / m_kRoundSideSize][i];
		avg_line_brightness /= (i - y0 / m_kRoundSideSize);
		return (avg_line_brightness <= m_max_line_brightness);
	}

	if (not zerkalim)
	{
		if (direction_x == 1)
		{
			for (int x = x0; x < x1; x += m_kRoundSideSize)
			{
				avg_line_brightness += m_SquaresBrightness[x / m_kRoundSideSize][y];
				square_counter += 1;
				error += tgA;
				if (error >= 1.0)
				{
					y += 1;
					error -= 1.0;
				}
			}
		}
		else
		{
			for (int x = x0; x > x1; x -= m_kRoundSideSize)
			{
				avg_line_brightness += m_SquaresBrightness[x / m_kRoundSideSize][y];
				square_counter += 1;
				error += tgA;
				if (error >= 1.0)
				{
					y += 1;
					error -= 1.0;
				}
			}
		}
	}
	else
	{
		if (direction_x == 1)
		{
			for (int x = x0; x < x1; x += m_kRoundSideSize)
			{
				avg_line_brightness += m_SquaresBrightness[y][x / m_kRoundSideSize];
				square_counter += 1;
				error += tgA;
				if (error >= 1.0)
				{
					y += 1;
					error -= 1.0;
				}
			}
		}
		else
		{
			for (int x = x0; x > x1; x -= m_kRoundSideSize)
			{
				avg_line_brightness += m_SquaresBrightness[y][x / m_kRoundSideSize];
				square_counter += 1;
				error += tgA;
				if (error >= 1.0)
				{
					y += 1;
					error -= 1.0;
				}
			}
		}
	}

	avg_line_brightness /= m_kSquaresHeightCount;
	return (avg_line_brightness <= m_max_line_brightness);
}

void LinesFilter::FrameHandler::DrawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t* out, const uint32_t* in) const
{
	 //https://en.wikipedia.org/wiki/Xiaolin_Wu%27s_line_algorithm

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
			plot(ypx11, xpx11, rfpart(yend) * xgap, out, in);
			plot(ypx11 + 1, xpx11, fpart(yend) * xgap, out, in);
		}
		else {
			plot(xpx11, ypx11, rfpart(yend) * xgap, out, in);
			plot(xpx11, ypx11 + 1, fpart(yend) * xgap, out, in);
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
			plot(ypx12, xpx12, rfpart(yend) * xgap, out, in);
			plot(ypx12 + 1, xpx12, fpart(yend) * xgap, out, in);
		}
		else {
			plot(xpx12, ypx12, rfpart(yend) * xgap, out, in);
			plot(xpx12, ypx12 + 1, fpart(yend) * xgap, out, in);
		}
	}

	if (steep) {
		for (int x = xpx11 + 1; x < xpx12; x++) {
			plot(ipart(intery), x, rfpart(intery), out, in);
			plot(ipart(intery) + 1, x, fpart(intery), out, in);
			intery += gradient;
		}
	}
	else {
		for (int x = xpx11 + 1; x < xpx12; x++) {
			plot(x, ipart(intery), rfpart(intery), out, in);
			plot(x, ipart(intery) + 1, fpart(intery), out, in);
			intery += gradient;
		}
	}
}


void LinesFilter::FrameHandler::plot(int32_t x, int32_t y, float brightness, uint32_t* out, const uint32_t* in) const
{
	if (*(reinterpret_cast<const uint8_t*>(in + x + m_kWidth * y) + 0) > m_max_line_brightness) return;
	*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 0) *= (1. - brightness);
	*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 1) *= (1. - brightness);
	*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 2) *= (1. - brightness);
	*(reinterpret_cast<uint8_t*>(out + x + m_kWidth * y) + 3) = 255;
}

frei0r::construct<LinesFilter> plugin("Straight lines",
	"kwa kwa",
	"rautyrauty",
	3, 0,
	F0R_COLOR_MODEL_RGBA8888);
