struct BIT {
    int a[MAX] = {0}, t[MAX] = {0}, N;

    int sum(int n) {
        int s = 0;
        while(n > 0) {
            s += t[n];
            n -= n & -n;
        }
        return s;
    }

    void add(int n, int d) {
        while(n <= N){
            t[n] += d;
            n += n & -n;
        }
    }

    int query(int a, int b) {
        return sum(b) - sum(a - 1);
    }
};
