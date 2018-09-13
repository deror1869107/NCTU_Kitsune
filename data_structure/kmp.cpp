vector<int> build_fail(const string& s){
    vector<int> fail;
    fail.push_back(-1);
    for(int i = 1, j = -1; i < (int) s.size(); i++){
        while(j >= 0 && s[j+1] != s[i])
            j = fail[j];
        if(s[j+1] == s[i])
            j++;
        fail.push_back(j);
    }
    return fail;
}

vector<int> match(const string& a, const string& b, const vector<int> &fail){
    vector<int> match_point;
    for(int i = 0, j = -1; i < (int) a.size(); i++){
        while(j >= 0 && b[j+1] != a[i])
            j = fail[j];
        if(b[j+1] == a[i])
            j++;
        if(j == (int) b.size() - 1){
            match_point.push_back(i - b.size() + 1);
            j = fail[j];
        }
    }
    return match_point;
}

// DLC: (ans[i] + 1) is the number of occurrence of prefix until index i in the original string
// for(int i = fail.size() - 1; i >= 0; --i) if(fail[i] != -1) ans[fail[i]] += ans[i] + 1;
