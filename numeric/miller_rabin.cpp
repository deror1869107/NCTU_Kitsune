typedef unsigned long long int ULL;

ULL pow(ULL a, ULL x, ULL n) // a ^ x % n
{
  ULL res = 1;
  while(x) {
    if(x & 1) res = res * a % n;
    a = a * a % n;
    x >>= 1;
  }
  return res;
}

bool miller_rabin(ULL a, ULL n) {
  if(n == a) return true;
  a %= n;
  if(a == 1 || a == n - 1) return true;
  ULL u = n - 1;
  while(!(u & 1)) u >>= 1;
  ULL x = pow(a, u, n);
  if(x == 1 || x == n - 1) return true;
  while(u < n - 1) {
    x = x * x % n;
    if(x == 1) return false;
    if(x == n - 1) return true;
    u <<= 1;
  }
  return false;
}

bool isprime(ULL n) {
  if(n < 2) return false;
  // ULL SPRP[7] = {2, 325, 9375, 28178, 450775, 9780504, 1795265022};
  int SPRP[3] = {2, 7, 61};
  for(int i = 0; i < 3; ++i) {
    if(!miller_rabin(SPRP[i], n)) return false;
  }
  return true;
}
