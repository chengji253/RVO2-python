import math


RVO_EPSILON = 0.00001


class Vector2:
      # A small constant for comparison, similar to C++ version

    def __init__(self, x=0.0, y=0.0):
        self.x_ = x
        self.y_ = y

    def __repr__(self):
        return f"({self.x_}, {self.y_})"

    def __neg__(self):
        return Vector2(-self.x_, -self.y_)

    def __mul__(self, other):
        if isinstance(other, Vector2):
            # Dot product
            return self.x_ * other.x_ + self.y_ * other.y_
        elif isinstance(other, (int, float)):
            # Scalar multiplication
            return Vector2(self.x_ * other, self.y_ * other)
        else:
            raise TypeError(f"Unsupported operand type(s) for *: 'Vector2' and '{type(other)}'")

    def __rmul__(self, other):
        # 反向乘法，支持 scalar * Vector2
        if isinstance(other, (int, float)):
            return Vector2(self.x_ * other, self.y_ * other)
        else:
            raise TypeError(f"Unsupported operand type(s) for *: '{type(other)}' and 'Vector2'")

    def __truediv__(self, scalar):
        inv_scalar = 1.0 / scalar
        return Vector2(self.x_ * inv_scalar, self.y_ * inv_scalar)

    def __add__(self, other):
        return Vector2(self.x_ + other.x_, self.y_ + other.y_)

    def __sub__(self, other):
        return Vector2(self.x_ - other.x_, self.y_ - other.y_)

    def __eq__(self, other):
        return self.x_ == other.x_ and self.y_ == other.y_

    def __ne__(self, other):
        return self.x_ != other.x_ or self.y_ != other.y_

    def __iadd__(self, other):
        self.x_ += other.x_
        self.y_ += other.y_
        return self

    def __isub__(self, other):
        self.x_ -= other.x_
        self.y_ -= other.y_
        return self

    def __imul__(self, scalar):
        self.x_ *= scalar
        self.y_ *= scalar
        return self

    def __idiv__(self, scalar):
        inv_scalar = 1.0 / scalar
        self.x_ *= inv_scalar
        self.y_ *= inv_scalar
        return self


def abs(vector):
    return math.sqrt(vector * vector)


def absSq(vector):
    return vector * vector


def det(vector1, vector2):
    return vector1.x_ * vector2.y_ - vector1.y_ * vector2.x_


def leftOf(vector1, vector2, vector3):
    return det(vector1 - vector3, vector2 - vector1)


def normalize(vector):
    return vector / abs(vector)


# Scalar multiplication for reverse operator
def scalar_multiply(scalar, vector):
    return Vector2(scalar * vector.x_, scalar * vector.y_)


if __name__ == "__main__":
    v1 = Vector2(3, 4)
    v2 = Vector2(1, 2)
    v3 = Vector2(4, 2)

    print(v1 + v2)          # 向量加法
    print(v1 - v2)          # 向量减法
    print(v1 * v2)          # 点积
    print(v1 * 2)
    print(scalar_multiply(2, v1))# 标量乘法
    print(v1 / 2)           # 标量除法
    print(abs(v1))  # 向量长度
    print(absSq(v1))
    print(det(v1, v2))  # 行列式（叉积）
    print(leftOf(v1, v2, v3))
    print(normalize(v1))
    # 使用外部函数进行标量乘法
    print(scalar_multiply(2, v1))  # 标量乘法

