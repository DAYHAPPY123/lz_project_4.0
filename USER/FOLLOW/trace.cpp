#include "trace.h"
#include <cstdlib>
#include "usart.h"

float x1_value[8] = {0, 14.3, 28.6, 42.9, 57.1, 71.4, 85.7, 100};
float y1_value[8] = {20, 24, 57, 89, 105, 81, 34, 29};
float x2_value[21] = {0};
float y2_value[21] = {0};

static int spline(int n, int end1, int end2,
                  float slope1, float slope2,
                  float x[], float y[],
                  float b[], float c[], float d[],
                  int *iflag)
{
    int nm1, ib, i, ascend;
    float t;
    nm1 = n - 1;
    *iflag = 0;
    if (n < 2)
    { /* no possible interpolation */
        *iflag = 1;
        goto LeaveSpline;
    }
    ascend = 1;
    for (i = 1; i < n; ++i)
        if (x[i] <= x[i - 1])
            ascend = 0;
    if (!ascend)
    {
        *iflag = 2;
        goto LeaveSpline;
    }
    if (n >= 3)
    {
        d[0] = x[1] - x[0];
        c[1] = (y[1] - y[0]) / d[0];
        for (i = 1; i < nm1; ++i)
        {
            d[i] = x[i + 1] - x[i];
            b[i] = 2.0f * (d[i - 1] + d[i]);
            c[i + 1] = (y[i + 1] - y[i]) / d[i];
            c[i] = c[i + 1] - c[i];
        }
        /* ---- Default End conditions */
        b[0] = -d[0];
        b[nm1] = -d[n - 2];
        c[0] = 0.0f;
        c[nm1] = 0.0f;
        if (n != 3)
        {
            c[0] = c[2] / (x[3] - x[1]) - c[1] / (x[2] - x[0]);
            c[nm1] = c[n - 2] / (x[nm1] - x[n - 3]) - c[n - 3] / (x[n - 2] - x[n - 4]);
            c[0] = c[0] * d[0] * d[0] / (x[3] - x[0]);
            c[nm1] = -c[nm1] * d[n - 2] * d[n - 2] / (x[nm1] - x[n - 4]);
        }
        /* Alternative end conditions -- known slopes */
        if (end1 == 1)
        {
            b[0] = 2.0f * (x[1] - x[0]);
            c[0] = (y[1] - y[0]) / (x[1] - x[0]) - slope1;
        }
        if (end2 == 1)
        {
            b[nm1] = 2.0f * (x[nm1] - x[n - 2]);
            c[nm1] = slope2 - (y[nm1] - y[n - 2]) / (x[nm1] - x[n - 2]);
        }
        /* Forward elimination */
        for (i = 1; i < n; ++i)
        {
            t = d[i - 1] / b[i - 1];
            b[i] = b[i] - t * d[i - 1];
            c[i] = c[i] - t * c[i - 1];
        }
        /* Back substitution */
        c[nm1] = c[nm1] / b[nm1];
        for (ib = 0; ib < nm1; ++ib)
        {
            i = n - ib - 2;
            c[i] = (c[i] - d[i] * c[i + 1]) / b[i];
        }
        b[nm1] = (y[nm1] - y[n - 2]) / d[n - 2] + d[n - 2] * (c[n - 2] + 2.0f * c[nm1]);
        for (i = 0; i < nm1; ++i)
        {
            b[i] = (y[i + 1] - y[i]) / d[i] - d[i] * (c[i + 1] + 2.0f * c[i]);
            d[i] = (c[i + 1] - c[i]) / d[i];
            c[i] = 3.0f * c[i];
        }
        c[nm1] = 3.0f * c[nm1];
        d[nm1] = d[n - 2];
    }
    else
    {
        b[0] = (y[1] - y[0]) / (x[1] - x[0]);
        c[0] = 0.0f;
        d[0] = 0.0f;
        b[1] = b[0];
        c[1] = 0.0f;
        d[1] = 0.0f;
    }
    LeaveSpline:
    return 0;
}

static float seval(int ni, float u,
                   int n, float x[], float y[],
                   float b[], float c[], float d[],
                   int *last)
{
    int i, j, k;
    float w;
    i = *last;
    if (i >= n - 1)
        i = 0;
    if (i < 0)
        i = 0;
    if ((x[i] > u) || (x[i + 1] < u)) //??
    {
        i = 0;
        j = n;
        do
        {
            k = (i + j) / 2;
            if (u < x[k])
                j = k;
            if (u >= x[k])
                i = k;
        } while (j > i + 1);
    }
    *last = i;
    w = u - x[i];
    w = y[i] + w * (b[i] + w * (c[i] + w * d[i]));
    return (w);
}

void SPL(int n, float *x, float *y, int ni, float *xi, float *yi)
{
    float *b, *c, *d;
    int iflag = 0, last = 0, i = 0;
    b = (float *)malloc(sizeof(float) * n);
    c = (float *)malloc(sizeof(float) * n);
    d = (float *)malloc(sizeof(float) * n);
    if (!d)
    {
        usart_printf("no enough memory for b,c,d\n");
    }
    else
    {
        spline(n, 0, 0, 0, 0, x, y, b, c, d, &iflag);
        for (i = 0; i < ni; i++)
            yi[i] = seval(ni, xi[i], n, x, y, b, c, d, &last);
        free(b);
        free(c);
        free(d);
    };
}

