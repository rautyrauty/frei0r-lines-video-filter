#include <iostream>
using namespace std;
int main()
{
	uint32_t** b = new uint32_t * [5];
	for (int i = 0; i < 5; i += 1)
	{
		b[i] = new uint32_t [10];
		std::fill(b[i], b[i] + 10, '0');
	}
	for (int i = 0; i < 5; i += 1)
	{
		for (int j = 0; j < 10; j += 1)
		{
			cout << b[i][j] << '\t';
		}
		cout << '\n';
	}
	return 0;
}