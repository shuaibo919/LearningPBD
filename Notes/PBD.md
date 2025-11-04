PBD最重要的准则：系统内部不进行力的考虑，只进行约束的构建
> - Müller, Matthias, et al. "Position based dynamics." Journal of Visual Communication and Image Representation 18.2 (2007): 109-118. 
> - Macklin, Miles, Matthias Müller, and Nuttapong Chentanez. "XPBD: position-based simulation of compliant constrained dynamics." Proceedings of the 9th International Conference on Motion in Games. 2016.
> - Müller, Matthias, et al. "Detailed rigid body simulation with extended position based dynamics." Computer Graphics Forum. Vol. 39. No. 8. 2020.

### 1-PBD概念
PBD构建一个模拟物体只需要三个表示：顶点，约束，以及每个顶点的状态
- 一个动态物体用$N$个顶点，$M$个约束表示，每个顶点的状态可能包含如下的属性：对于$i\in [0,...N-1]$，其可能具有质量$m_i$，位置$x_i$，速度$v_i$等

该动态物体的形状是基于约束进行限制的，而不是基于经典的力学模型。
而对整个系统外的外力仍然需要进行牛顿形式的运动学计算。对于布料来说，结点与结点之间的力就不需要考虑啦（要求的话考虑Mass Spring或者Fem这样的模型）
那么，什么是约束？以下给出约束的一个例子：
$$C(X) = |x_0-x_1|-d$$
若某动态物体只有两个顶点，希望该物体的这两个顶点保持距离$$d$$,那么上面的约束$$C(X)$$就描述了这一行为。（有点损失函数的感觉）。
对于抽象的情况，一个系统可能具有$m$个约束：
$$C(P)=\begin{bmatrix} C_0(x_0,x_1,....x_{n-1}) \\ C_1(x_0,x_1,....x_{n-1}) \\. \\. \\C_{m-1}(x_0,x_1,....x_{n-1})  \end{bmatrix}$$
其中，$P = \{x_0,x_1,...x_{n-1}\}, C = \{C_0(P),C_1(P),..,C_{m-1}(P)\}^T$

下面，需要处理的问题是当一个系统被施加外力后，我们通过约束去修改原来的位置。为了使得能量守恒，需要保证系统内的约束满足最基本的角动量与线动量守恒，这里证明略。

一个观察是我们希望应用约束修正后，约束条件不变：$C(P +\Delta P) = 0$，在假设基础上应用一阶泰勒展开
$$C(P +\Delta P) \approx C(P) + \nabla_P C(P) \cdot \Delta P = 0$$
这里还有一个假设，即限制$\Delta P$必须沿着梯度方向：
$$\Delta P = \nabla_P C(P) \cdot \lambda$$

解得$\lambda = \frac{-C(P)}{|\nabla_P C(P)|^2}$，于是$\Delta P = \frac{-C(P)}{|\nabla_P C(P)|^2}\nabla_P C(P)$

对于质量不相等的情况，$\Delta P$需要修改为$\Delta P = M^{-1}\nabla_P C(P) \cdot \lambda$, $M$是对角阵，主对角线元素为质量${m_0,m_1,...,m_{n-1}}$。类似的，可以得到质量不均匀情况下的$\Delta P$：
$\Delta P_{i} = -w_i\nabla_{P_{i}} C(P)\cdot \frac{C(P)}{\sum_j w_j |\nabla_{P_{j}}C(P)|^2}$

### 2-应用
#### 2.1 距离约束
对于基础的、仅有两个位置点的一个距离约束$|P_1-P_2|-d$，我们可以求得
$$\nabla_{P_1} C(P) = \frac{P_1-P_2}{|P_1-P_2|}, \nabla_{P_2} C(P) = \frac{P_2-P_1}{|P_1-P_2|}$$
$$\Delta P_1 = -w_1\frac{P_1-P_2}{|P_1-P_2|} \cdot \frac{|P_1-P_2|-d}{w_1+w_2} = \frac{-w_1}{w_1+w_2}(|P_1-P_2|-d)\frac{P_1-P_2}{|P_1-P_2|}$$
$$ \Delta P_2 = \frac{-w_2}{w_1+w_2}(|P_1-P_2|-d)\frac{P_2-P_1}{|P_1-P_2|}$$


以上图为例，若$|P_1-P_2|> d$, $\Delta P_1 $的方法为$P_2P_1$的反方向，等同于将$P_1$往原有的位置拉回。
#### 2.2 布料模拟约束
- 对每个边的Stretch约束: $C_{stretch}(P_1,P_2)=|P1-P2|-l_0$，$l_0$是初始的距离
- 对两个相邻的三角形，使用Bend约束$C_{bend}(P_1,P_2,P_3,P_4)=$
$$acos(\frac{(P_2-P_1) \times (P_3-P_1)}{|(P_2-P_1) \times (P_3-P_1)|} \cdot \frac{(P_2-P_1) \times (P_4-P_1)}{|(P_2-P_1) \times (P_4-P_1)|}) - \phi_0$$
对于Bend约束，其实就是取两个相邻三角形的法线的点积的acos并与预设的$$\phi_0$$比较，两平面法线的点积是两个法线向量长度与它们夹角余弦的乘积，Bend约束就是要约束两个平面的角度。


- 对于自碰撞约束，其描述一个顶点$q$穿过一个三角形$P_0,P_1,P_2$时的行为
$(q-P_1)\cdot\frac{(P_2-P_1) \times (P_3-P_1)}{|(P_2-P_1) \times (P_3-P_1)|} - h_{thickness}$


#### 2.3 碰撞约束
对于碰撞约束，PBD采取的方式是从顶点发射一根射线$x_i →p_i$，如果该射线进入了某个物体，那么就求出交点$q_{c}$以及该点处的法线$n_c$：

碰撞的约束方程为$C(p)=(p-q_c)\cdot n_c \geq 0$。如果射线已完全被物体包围，那么以上方法失效。需要计算离$p$点最近的表面点$q_s$,以及该点的法线$n_s$来替换上述约束方程中的$q_c$和$n_c$。

---
更多约束参考原论文。

### 3-PBD进化：XPBD
3.1 牛顿第二定律与势能的引入
XPBD，对PBD的拓展工作。主要解决Stiffness和时间步、迭代次数不正交的问题。
XPBD引入牛顿第二定律$$F=ma$$和势能的概念：
$$M\ddot{X} = -\nabla U^T(X)$$
其中$M$是质量，$\ddot{X}$代表位置的二阶导数，即加速度，$\nabla U^T(X)$是势能的导数，即保守力，当保守力为0时，保持稳定平衡，在保守力场中，力的方向总是沿着势能的负梯度方向。
对于游戏中的实时模拟，上述式子需要转为离散表达以进行程序模拟：
$M(\frac{x_{n+1}-2x_{n}+x_{n-1}}{\Delta t^2})=  -\nabla U^T(X_{n
+1})$
其中势能使用弹性势能的公式$U=\frac{1}{2}k\Delta X^2$, $k$为弹性系数，$\Delta X^2$为形变量。

---
#### 3.2 势能的PBD约束形式
转换为PBD的语言描述，$U=\frac{1}{2}k C(X)^2$为单个约束的弹性势能，当约束满足时，弹性势能为0。
对于多个约束的情况，转换为矩阵形式：
$$U = \frac{1}{2}C(X)^T\alpha^{-1} C(X)$$
其中，$\alpha$是对角矩阵, 其对角线元素为${\alpha_0, \alpha_1, ...., \alpha_{m-1}}$，表示Stiffness。
现在可以求$\nabla U^T(X)$了, 
$$\nabla U^T(X) = \frac{1}{2}  \nabla  \begin{bmatrix} 
&\alpha_{0}^{-1}C_0(X)^2, & 0,              &0,&..., &0  \\ 
&0,                &\alpha_{1}^{-1}C_1(X)^2,&0,&...,&0  \\
&&&...... \\
&0,                &0, &0,&...,&\alpha_{m-1}^{-1}C_{m-1}(X)^2  \\
\end{bmatrix}$$

$$\nabla U^T(X) =\nabla  C^T(X) \alpha^{-1}  C(X)$$

---
#### 3.3 转换为迭代形式
将上述结果带入$M(\frac{x_{n+1}-2x_{n}+x_{n-1}}{\Delta t^2})=  -\nabla U^T(X_{n
+1})$ , 有
$$M(\frac{X_{n+1}-2X_{n}+X_{n-1}}{\Delta t^2})=  -\nabla C^T(X_{n
+1}) \alpha^{-1}  C(X_{n
+1})$$

这里，XPBD使用了一个精髓的假设，将左式的 $\Delta t^2$ 移到了右式，并令
$\lambda_{elastic} = -(\frac{\alpha}{\Delta t^2})^{-1}C(X)$
这样可以得到两个等式：
$$g(X,\lambda) = M(X_{n+1} - \hat{X}) - \nabla C(X_{n
+1})^T\lambda_{n+1} = 0$$
$$h(X,\lambda) = C(X_{n
+1}) + \frac{\alpha}{\Delta t^2 }\lambda_{n+1} = 0$$

其中$\hat{X} = X_{n} + X_{n} - X_{n-1} = X_{n} + \Delta tV_n$，后记$\hat{\alpha} = \frac{\alpha}{\Delta t^2 }$

到目前为止，只是一系列变换，解上述的方程和一开始的式子难度并无变化
对$h(X,\lambda)$和$g(X,\lambda)$应用一阶离散泰勒展开进行近似

- 对$h(X,\lambda)$的近似展开
$$h(X_i,\lambda_i) + \nabla C(x_i) \Delta X_i + \hat{\alpha} \Delta \lambda_i = 0$$

- 对$g(X,\lambda)$的近似展开
$$\frac{\partial g(X_i,\lambda_i)}{\partial X_i} = M - \nabla^2 C(X_i)^T \lambda_i, \quad \frac{\partial g(X_i,\lambda_i)}{\partial \lambda_i} = - \nabla C(X_{i})^T $$
$$g(X_i,\lambda_i) + (M - \nabla^2 C(X_i)^T \lambda_i) \Delta X_i - \nabla C(X_{i})^T  \Delta \lambda_i    = 0$$

为了简化操作，省取自变量和下标，并令 $K = (M - \nabla^2 C(X_i)^T \lambda_i)$，
 $h$和$g$变为如下形式：
$$h+\nabla C \Delta X + \hat{\alpha} \Delta \lambda = 0$$
$$g + K\Delta X - \nabla C^T \Delta \lambda = 0$$
这里其实就对应原论文的式12：


#### 3.4 两个精髓的假设
上述关于的两个方程可以解出$\Delta X$和$\Delta \lambda$：
$$\Delta \lambda = (\nabla C^T K^{-1}\nabla C +\hat{\alpha})^{-1}(gK^{-1}\nabla C - h)$$
$$\Delta X = K^{-1} (\nabla C^T\Delta \lambda - g)$$

现在如果知道$X_i$和$\lambda_i$，把值代入上式就可以迭代计算（对应论文的式13和式14）：


现在的问题是里面的K，包含了约束方程的二阶梯度，这个Hessian矩阵求起来复杂度很高。这里原论文又给了一个精髓的假设，令$K = M$, 忽略后面的 $\nabla^2 C(X_i)^T \lambda_i$，这可能影响收敛速度，但不对全局误差造成本质影响
 

第二个精髓的假设是$g = 0$, 显然当处于Rest状态时是满足的，但后续不一定满足，这里作者的解释是：

3.5 开始迭代！
经过一系列操作后我们可以得到最终的$\Delta X$和$\Delta \lambda$：
$$\Delta \lambda = -(\nabla C^T M^{-1}\nabla C +\hat{\alpha})^{-1}(C +\hat{\alpha}\lambda)$$
$$\Delta X = M^{-1} \nabla C^T\Delta \lambda $$
这就是最终的迭代公式。

3.6 Damping


4-使用XPBD进行刚体模拟
Detailed Rigid Body Simulation with Extended Position Based Dynamics
