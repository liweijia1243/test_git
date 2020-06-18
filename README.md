<h1 align="center"> ✨ Markdown Preview forWeijia's first MD ✨ </h1>
# 目录

![screenshot](https://user-images.githubusercontent.com/5492542/47603494-28e90000-da1f-11e8-9079-30646e551e7a.gif)

-----
${toc}
[[toc]]
[toc]
[[_toc_]]

---
# 表格
``` plantuml
Bob -> Alice : hello
```
| name     |     address     |       phone |
|:---------|:---------------:|------------:|
| liweijia | jiadingxincheng | 18817871243 |
| limingyu |   oudongxinqu   |   186sddssf |
| 李伟嘉   |      永远爱     |        宝宝 |
## Code
``` c++
#include <iostream>
#include <string>
using namespace std;
main()
{
    for (int i=0;i<100;i++)
    {
        cout<<"hello world"<<endl;
    }
}
```
# 第二部分
## 字体
### 加粗
1.**这是加粗字体**

2.*这是倾斜字体*

3.***这是倾斜加粗字体***

4.~~这是加删除线的字体~~
## 引用
>这是引用的内容
>>这是引用的内容
>>>>>>>>>>这是引用的内容
## 分割线
1.三个或者三个以上的 - 或者 * 都可以。

---
----
***
*****

效果如下:

可以看到，显示效果是一样的。

## 图片
![图片alt](图片地址 ''图片title'')

图片alt就是显示在图片下面的文字，相当于对图片内容的解释。

图片title是图片的标题，当鼠标移到图片上时显示的内容。title可加可不加

![blockchain](https://upload-images.jianshu.io/upload_images/6860761-fd2f51090a890873.jpg?imageMogr2/auto-orient/strip|imageView2/2/format/webp
u=702257389,1274025419&fm=27&gp=0.jpg "区块链")

![blockchain](https://steamuserimages-a.akamaihd.net/ugc/924835275508550089/539770DFA42937BF327F58208ACCA10EA9912A60/?imw=637&imh=358&ima=fit&impolicy=Letterbox&imcolor=%23000000&letterbox=true)

## 超链接
[超链接名](超链接地址 "超链接title")
title可加可不加

[简书](http://jianshu.com)
[百度](http://baidu.com)

注：Markdown本身语法不支持链接在新页面中打开，貌似简书做了处理，是可以的。别的平台可能就不行了，如果想要在新页面中打开的话可以用html语言的a标签代替。

<a href="https://www.jianshu.com/u/1f5ac0cf6a8b" target="_blank">简书</a>

## 列表
### 无序列表
语法：

无序列表用 - + * 任何一种都可以
- 列表内容
- 列表内容
+ xxx
+ bbbb
* hhh
* 列表内容

注意：- + * 跟内容之间都要有一个空格
### 有序列表
语法：
数字加点
1. 列表内容
3. 列表内容
4. 列表内容

注意：序号跟内容之间要有空格

### 列表嵌套
上一级和下一级之间敲三个空格即可
1. 一级表格
   - hhh
   - hhh
   - cc
2. 内容
   1. cc
   2. cc 
      * ss
      * ff
         1. 111
         2. 222

### 表格
表头|表头|表头
---|:--:|---:
内容|内容|内容
内容|内容|内容

第二行分割表头和内容。
- 有一个就行，为了对齐，多加了几个
文字默认居左
-两边加：表示文字居中
-右边加：表示文字居右
注：原生的语法两边都要用 | 包起来。此处省略


姓名|技能|排行
--|:--:|--:
刘备|哭|大哥
关羽|打|二哥
张飞|骂|三弟

## 代码
语法：
单行代码：代码之间分别用一个反引号包起来

`代码内容 for (int i=0;i<100;i++){cout<<""<<endl;}`

代码块：代码之间分别用三个反引号包起来，且两边的反引号单独占一行

```
  代码...
  代码...
  代码...
```

```
    function fun(){
         echo "这是一句非常牛逼的代码";
    }
    fun();
```
## 流程图等多种图表
----
![image](https://user-images.githubusercontent.com/5492542/47603494-28e90000-da1f-11e8-9079-30646e551e7a.gif =400x200)
---
$\sqrt{3x-1}+(1+x)^2$

$$\begin{array}{c}

\nabla \times \vec{\mathbf{B}} -\, \frac1c\, \frac{\partial\vec{\mathbf{E}}}{\partial t} &= \frac{4\pi}{c}\vec{\mathbf{j}}    \nabla \cdot \vec{\mathbf{E}} &= 4 \pi \rho \\

\nabla \times \vec{\mathbf{E}}\, +\, \frac1c\, \frac{\partial\vec{\mathbf{B}}}{\partial t} &= \vec{\mathbf{0}} \\

\nabla \cdot \vec{\mathbf{B}} &= 0

\end{array}$$

----
``` mermaid
gantt
    dateFormat DD-MM-YYY
    axisFormat %m/%y

    title Example
    section example section
    activity :active, 01-02-2019, 03-08-2019
```
----
``` sequence-diagrams
Andrew->China: Says
Note right of China: China thinks\nabout it
China-->Andrew: How are you?
Andrew->>China: I am good thanks!
```
----
``` flowchart
st=>start: Start|past:>http://www.google.com[blank]
e=>end: End|future:>http://www.google.com
op1=>operation: My Operation|past
op2=>operation: Stuff|current
sub1=>subroutine: My Subroutine|invalid
cond=>condition: Yes
or No?|approved:>http://www.google.com
c2=>condition: Good idea|rejected
io=>inputoutput: catch something...|future

st->op1(right)->cond
cond(yes, right)->c2
cond(no)->sub1(left)->op1
c2(yes)->io->e
c2(no)->op2->e
```
----

``` dot
digraph G {

  subgraph cluster_0 {
    style=filled;
    color=lightgrey;
    node [style=filled,color=white];
    a0 -> a1 -> a2 -> a3;
    label = "process #1";
  }

  subgraph cluster_1 {
    node [style=filled];
    b0 -> b1 -> b2 -> b3;
    label = "process #2";
    color=blue
  }
  start -> a0;
  start -> b0;
  a1 -> b3;
  b2 -> a3;
  a3 -> a0;
  a3 -> end;
  b3 -> end;

  start [shape=Mdiamond];
  end [shape=Msquare];
}
```
----
``` chart
{
  "type": "pie",
  "data": {
    "labels": [
      "Red",
      "Blue",
      "Yellow"
    ],
    "datasets": [
      {
        "data": [
          300,
          50,
          100
        ],
        "backgroundColor": [
          "#FF6384",
          "#36A2EB",
          "#FFCE56"
        ],
        "hoverBackgroundColor": [
          "#FF6384",
          "#36A2EB",
          "#FFCE56"
        ]
      }
    ]
  },
  "options": {}
}
```
----
