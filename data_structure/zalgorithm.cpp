/* 定義一個字串S的Z function:
 * Z(i)=0 如果i=0 or S[i]!=S[0]     否則
 * Z(i)=max(所有的k滿足 S[0,k-1]=S[i,i+k-1])
 * 從Z function的定義，假設要在字串A裡面匹配字串B
 * 可以先建構字串S=B+(A和B都沒有的字元)+A的Z function陣列
 * 若Z[i]=lenB則表示在A[i-lenB]有一個完全匹配
 */
inline void z_alg1(char *s, int len, int *z){
    int l = 0, r = 0;
    z[0] = len;
    for(int i = 1; i < len; ++i){
        z[i] = r > i ? min(r - i + 1, z[z[l] - (r - i + 1)]) : 0;
        while(i + z[i] < len && s[z[i]] == s[i + z[i]]) ++z[i];
        if(i + z[i] - 1 > r) r = i + z[i] - 1, l = i;
    }
}
