#ifndef maths_h
#define maths_h
#include <Arduino.h>



#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))
// Use floating point M_PI instead explicitly.
#define M_PIf       3.14159265358979323846f
#define RAD    (M_PIf / 180.0f)
#define DEGREES_TO_RADIANS(angle) ((angle) * RAD)
#define DEG2RAD    0.017453293f  /* 度转弧度 π/180 */
#define RADIANS_TO_DEGREES(angle) ((angle) / RAD)

#define sinPolyCoef3 -1.666665710e-1f                                          // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5  8.333017292e-3f                                          // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f                                          // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9  2.600054768e-6f  


typedef struct stdev_s
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;



float sin_approx(float x);
float cos_approx(float x);
float constrainf(float amt, float low, float high);
void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devStandardDeviation(stdev_t *dev);

#endif
