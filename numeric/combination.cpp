const int M = 1000000007;
int* c[20001];

void init() {
  int r[2][20005] = {0};
  r[0][0] = 1;
  for(int i = 1; i < 20001; ++i) {
    r[i & 1][0] = r[i & 1][i] = 1;
    for(int j = 1; j < i; ++j) {
      long long int k = r[(i - 1) & 1][j - 1] + r[(i - 1) & 1][j];
      r[i & 1][j] = k % M;
    }
    c[i] = new int[i / 2 + 1];
    for(int k = 0; k <= i / 2; ++k) {
      c[i][k] = r[i & 1][k];
    }
  }
}

int C(int n, int k) {
  return (n - k < k) ? c[n][n - k] : c[n][k];
}
