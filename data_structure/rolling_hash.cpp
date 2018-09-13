const int B = 17, M = 1000000007, MAXN = 1048576;
long long Bpower[MAXN];

void build_Bpower(){
    Bpower[0] = 1;
    for(int i = 1; i < MAXN; i++)
        Bpower[i] = (Bpower[i-1] * B) % M;
}

vector<int> rolling_hash(const string &s){
    vector<int> v;
    v.push_back(0);
    for(int i = 0; i < (int) s.size(); i++)
        v.push_back((v.back() + s[i] * Bpower[i]) % M);
    return v;
}

inline bool same(int bhv,const vector<int> &ahv_list, int k, int l){
    return bhv * Bpower[k] % M == (ahv_list[k+l] - ahv_list[k] + M) % M;
}

vector<int> match(int bhv, const vector<int> &ahv_list, int b_size){
    vector<int> match_point;
    for(int i = 0; i < (int) ahv_list.size() - b_size; i++)
        if(same(bhv, ahv_list, i, b_size))
            match_point.push_back(i);
    return match_point;
}
