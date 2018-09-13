vector<int> prime;
bool sieve[46341] = {false};  // false: prime

void build() {
    for(int i = 2; i < 46341; ++i) {
        if(!sieve[i]) {
            prime.push_back(i);
            for(int j = i * i; j < 46341; j += i) {
                sieve[j] = true;
            }
        }
    }
}
