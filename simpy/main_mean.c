#include <stdio.h>
#include "calc_mean.h"

int main(int argc, char* argv[])
{
	double v1, v2, m;
	v1 = 5.234;
	v2 = 3.567;

	m = mean(v1, v2);

	printf(" mean of %3.2f and %3.2f is %3.2f\n", v1, v2, m);

	return 0;
}
