#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>



void process_fft_input(double* m_fft_input_f64, int n) {
    int i, j = 0;

    for (i = 2; i < n; i += 2) {
        m_fft_input_f64[j] = m_fft_input_f64[i];
        j++;
    }
}
void roll_signal(float *signal, int n, int shift) {
    float temp[n];
    int abs_shift = abs(shift) % n;
    
    if (shift > 0) {
        memcpy(temp, signal + n - abs_shift, abs_shift * sizeof(float));
        memmove(signal + abs_shift, signal, (n - abs_shift) * sizeof(float));
        memcpy(signal, temp, abs_shift * sizeof(float));
    } else if (shift < 0) {
        memcpy(temp, signal, abs_shift * sizeof(float));
        memmove(signal, signal + abs_shift, (n - abs_shift) * sizeof(float));
        memcpy(signal + n - abs_shift, temp, abs_shift * sizeof(float));
    }
}


int main() {
    int i;
    int n = 10;
    float m_fft_input_f64[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    printf("Before processing:\n");
    for (i = 0; i < n; i++) {
        printf("%.0f ", m_fft_input_f64[i]);
    }
    printf("\n");

    roll_signal(m_fft_input_f64, n,2);

    printf("After processing:\n");
    for (i = 0; i < n; i++) {
        printf("%.0f ", m_fft_input_f64[i]);
    }
    printf("\n");

    return 0;
}