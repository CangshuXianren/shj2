#ifndef SH_UTIL
#define SH_UTIL

#define RESET "\033[0m"
#define BLACK "\033[30m"     /* Black */
#define RED "\033[1;31m"     /* Red */
#define GREEN "\033[1;32m"   /* Green */
#define YELLOW "\033[1;33m"  /* Yellow */
#define BLUE "\033[1;34m"    /* Blue */
#define MAGENTA "\033[1;35m" /* Magenta */
#define CYAN "\033[1;36m"    /* Cyan */
#define WHITE "\033[1;37m"   /* White */

template <typename T1, typename T2> inline void CoordinateConversion(const T1& pi, T1& po, T2& test) {
    po[0] = pi[0] * cos(test[2]) - pi[1] * sin(test[2]) + test[0];
    po[1] = pi[0] * sin(test[2]) + pi[1] * cos(test[2]) + test[1];
}

#endif