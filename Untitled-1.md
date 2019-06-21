

---
export_on_save:
 html: true
---


共同筆記
===
- [本網頁](https://hackmd.io/@derrick/shut1)
- [TOC]

1111
===
## Keynote
### 09:40 ~ 10:20
- [A Modern Web Architecture for (GDPR) Compliance - 曾義峰 (Ant)]

## Track A
### 11:30 ~ 11:55
- [傳統如何翻新，用Serverless新架構改造傳統資料分析ETL：以利用 AWS Lambda + Batch 為例 - 楊立偉 (Bryan Yang)]
- 圖片
 ![123](https://octodex.github.com/images/stormtroopocat.jpg)
 ![Alt text][id]

[id]: https://octodex.github.com/images/dojocat.jpg

## Track B
### 11:30 ~ 11:55
- [從前端切入以太坊區塊鏈 - 廖洧杰]


2222
===
## Keynote
### 09:40 ~ 10:20
- [The Layers Of The Web - Jeremy Keith]


## Track A
### 13:30 ~ 14:10
- [微服務了沒 - 陳鋒逸 (陳小風)]
- ## MathJax

您可以使用 **MathJax** 語法 來產生 *LaTeX* 數學表達式，如同 [math.stackexchange.com](http://math.stackexchange.com/)，但是開始的 `$` 後面以及結尾的 `$` 前面不能有空白：

The *Gamma function* satisfying $\Gamma(n) = (n-1)!\quad\forall n\in\mathbb N$ is via the Euler integral

$$
x = {-b \pm \sqrt{b^2-4ac} \over 2a}.
$$

$$
\Gamma(z) = \int_0^\infty t^{z-1}e^{-t}dt\,.
$$

> 更多關於 **LaTeX** 數學表達式 [請至這裡](http://meta.math.stackexchange.com/questions/5020/mathjax-basic-tutorial-and-quick-reference)


HackMD -- LaTeX 語法與示範
===

在接下來的課程作業會出現大量的數學式，而作業要求之一即 <font color="red">不使用圖片顯示文字訊息</font>，當然也包含了繁複的數學式。因此，以下我們會引導大家在 HackMD 中使用 [LaTeX](https://zh.wikipedia.org/zh-tw/LaTeX) 表達數學式。
:::info
右鍵點選
* 顯示出來並沒有`$`，但切記在 HackMD 中，必須以`$`包住才會認得是 LaTeX 語法喔!
:::

### 數學式的開始與結束都是`$`
* 如我們想表達: N(b, d) = (b − 1) M
    * 使用 LaTeX 後變成: $N(b,d)=(b-1)M$

另外，==`$`需緊鄰著數學式==，也就是中間不能有任何空白鍵，否則一樣無法辨識為 LaTeX 語法!
* 一樣以 $N(b,d)=(b-1)M$ 為例
    * `$`插入空白: 
        * $ N(b,d)=(b-1)M$ (開始的`$`後插入空白)
        * $N(b,d)=(b-1)M $ (結束的`$`前插入空白)
        * $ N(b,d)=(b-1)M $ (前後的`$`都插入空白)

值得高興的是，除了`$`以外的符號都可以加入空白以增加可讀性，而不影響顯示結果
透過右鍵看 LaTeX Code 可以看出下面兩個式子的差異
* 一樣以 $N(b,d)=(b-1)M$ 為例
    * 任意插入 $n$ 個空白: 
        * $N ( b ,      d )=( b - 1 )M$ 

## LaTex 排版
### 如何插入空白
雖然空白可以增加可讀性，但若我們真的希望顯示結果是有空白的話呢?

* 如我們希望用 LaTeX 表示: 7 months + 5 months = 1 year
    * 前後加上`$`後: $7 months + 5 months = 1 year$

完全不是預期的結果! 而且不論加上多少空白，還是會跟上面一模一樣......

**解決辦法為: 加入`\`**
這其實跟寫程式要印出特殊符號的方法一樣，就是再加一個`\` 
* 馬上試試看: $7\ months + 5\ months = 1\ year$
* $Lily\ brother$

完美呈現!!

:::info
`\`在 LaTeX 中常用到，當要加入特殊符號時一定會出現! 用以區分純英文單字和特殊符號
如:
* $alpha$ 與 $\alpha$
:::

### 如何插入換行
除了空白，排版不能或缺的當然還有換行囉！
換行：`\\`
如：以下數學式為加入換行排版後的呈現：
$ \dfrac{1}{2} $
$二分之一:\\ \dfrac{1}{2}$
### 如何對齊方程式
再次以同樣的式子為例，若沒加入對齊：
$\\where\ t_k = a + (b-a)\cdot k/N$

* **以等號對齊**: `\begin{split}...\end{split}`
    * 在方程式的開頭加上 `\begin{split}`，結尾加上 `\end{split}`，等號前加上 `&`
$\mathrm{Integrals\ are\ numerically\ approximated\ as\ finite\ series}:\\
\begin{split}
\int_{a}^{b}x(t)dt &= 
\dfrac{b - a}{N} \\
&\approx\sum_{k=1}^{N}x(t_k)\cdot\dfrac{b-a}{N}
\end{split} \\ 
where\ t_k = a + (b-a)\cdot k/N$
    * `&` 是標記從哪裡開始對齊的標記點
        * 你可以自由移動 `&` 改變對齊的符號實驗看看
 
* **多行切割**: `\begin{multline*}...\end{multline*}`
若式子太長，分段切割顯示才好閱讀，否則字會超出版面
中間要換行的地方一樣插入 `\\`
    * 沒換行：
$p(x) = 3x^6 + 14x^5y + 590x^4y^2 + 19x^3y^3 - 12x^2y^4 - 12xy^5 + 2y^6 - a^3b^3 - a^2b - ab + c^5d^3 + c^4d^3 - cd$
    * 有換行:
$\begin{multline*}p(x) = 3x^6 + 14x^5y + 590x^4y^2 + 19x^3y^3 \\ - 12x^2y^4 - 12xy^5 + 2y^6 - a^3b^3 - a^2b - ab + c^5d^3 + c^4d^3 - cd\end{multline*}$

* **置中對齊**: `\begin{gather*}...\end{gather*}`
    * 沒置中對齊：
$2x - 5y =  8 \\ 3x^2 + 9y =  3a + c$ 
    * 有置中對齊：
$\begin{gather*} 2x - 5y =  8 \\ 3x^2 + 9y =  3a + c\end{gather*}$



## 常用數學式
以下會列舉出在作業中可能常用到的數學式

### 次方`^`
想表達 a 的 b 次方: $ a ^ b $

如: $2^5 = 32$

當次方項多於一個字時，需使用`{}`包住
* 如: 2 的 10 次方
    * $2^10 = 1024$ (沒有使用`{}`)
    * $2^{10} = 1024$
* 文字: 2 的 n - 1 次方
    * $2^{n-1}$ 

當底數是多於一個字時
* 如:
$1.02^{365} = 1377.4$
$1.01^{365} = 37.8$
多百分之一的努力，得千分收穫!!一起加油!!


### 直式除法 `\dfrac{分子}{分母}`
想表達二分之一: $ \dfrac{1}{2} $
二分之一: $\dfrac{1}{2}$

`{}`一樣可以塞任意文字數字

### 循環小數 `\overline{循環部分}`
AB 循環: $ \overline{AB} $ 

`{}`填入循環部分
如: $0.00\overline{123}$

### 底標 `_`
$M_i$: $ M_i $

底標多個字時一樣使用 `{}`
$M_{this\ is\ a\ M}$

### 矩陣
表達一個 $3 \times 3$ 矩陣:
$\left(
\begin{array}{ccc}
1 & 2 & 3 \\
4 & 5 & 6 \\
7 & 8 & 9 \\
\end{array}
\right)$

* `\left(`: 
    * 左半邊括號，不一定要是`(`，也可以是`[` `|`
* `\begin{array}`:
    * 矩陣開始
* `{ccc}`:
    * 定義有幾個 column
* `1 & 2 & 3 \\`:
    * 依序填入矩陣內容，每行以 `\\` 結尾f
* `\end{array}`:
    * 矩陣結束
* `\right)`:
    * 右半邊括號，如左半邊，可以是別種類型的括號

### 積分 `\int_{a}^{b}`
$\int_{a}^{b}x(t)dt$: `\int_{a}^{b}x(t)dt`

### limit \lim_{N\to\infty}
有兩種呈現方式：
* $\lim_{N\to\infty}$
* $\displaystyle\lim_{N\to\infty}$
    * 加入 `displaystyle`

`\to` 即為 $\to$ 

### sums `\sum_{k=1}^{N}`
有兩種呈現方式：
* $D{k=1}^{N}x(t_k)$
    * `\sum_{k=1}^{N}x(t_k)`
* $\displaystyle\sum_{k=1}^{N}x(t_k)$
    * 多加入 `\displaystyle`

### multiple dot `\cdot`
如以下例子：
$\sum_{k=1}^{N}x(t_k)\cdot\dfrac{b - a}{N}$
須使用 `\cdot` 來表示，鍵盤上輸入 `.` 會是小數點 

### multiple x `\times`
若想再方程式中打上乘號(x)，若單純輸入 x 會被視為字母
如下：
* $a x b$: `a x b`
* $a \times b$: `a \times b`
* ![](http://latex.codecogs.com/gif.latex?\\frac{\\partial J}{\\partial \\theta_k^{(j)}}=\\sum_{i:r(i,j)=1}{\\big((\\theta^{(j)})^Tx^{(i)}-y^{(i,j)}\\big)x_k^{(i)}}+\\lambda \\xtheta_k^{(j)})



