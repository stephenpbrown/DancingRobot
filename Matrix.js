/**
 * Created by cs442 on 9/8/16.
 */

function Vec3(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
}

Vec3.prototype.dot = function(other) {
    return this.x*other.x + this.y*other.y + this.z*other.z;
};

Vec3.prototype.cross = function(other) {
    return new Vec3(this.y*other.z - this.z*other.y,
        this.z*other.x - this.x*other.z,
        this.x*other.y - this.y*other.x);
};

Vec3.prototype.mag2 = function() {
    return this.dot(this);
};

Vec3.prototype.mag = function() {return Math.sqrt(this.mag2());};

Vec3.prototype.normalize = function() {
    var s = 1/this.mag();
    this.x *= s;
    this.y *= s;
    this.z *= s;
};

Vec3.prototype.subtract = function(other) {
    return new Vec3(this.x - other.x,
        this.y - other.y,
        this.z - other.z);
};

function Matrix4x4() {
    this.array = [1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1];
}

Matrix4x4.prototype.elem = function(row, col) {
    return this.array[row + 4*col];
};

Matrix4x4.prototype.setElem = function(row, col, val) {
    this.array[row + 4*col] = val;
};

Matrix4x4.prototype.copy = function(other) {
    this.array = other.array.slice(0);
    return this;
};

Matrix4x4.prototype.identity = function() {
    this.array = [1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1];
};

Matrix4x4.prototype.set4x4 = function(a00, a01, a02, a03,
                                      a10, a11, a12, a13,
                                      a20, a21, a22, a23,
                                      a30, a31, a32, a33) {
    this.setElem(0,0, a00);
    this.setElem(0,1, a01);
    this.setElem(0,2, a02);
    this.setElem(0,3, a03);

    this.setElem(1,0, a10);
    this.setElem(1,1, a11);
    this.setElem(1,2, a12);
    this.setElem(1,3, a13);

    this.setElem(2,0, a20);
    this.setElem(2,1, a21);
    this.setElem(2,2, a22);
    this.setElem(2,3, a23);

    this.setElem(3,0, a30);
    this.setElem(3,1, a31);
    this.setElem(3,2, a32);
    this.setElem(3,3, a33);
};

Matrix4x4.prototype.mult = function(B) {
    var AB = new Matrix4x4();
    for (var r = 0; r < 4; r++) {
        for (var c = 0; c < 4; c++) {
            var s = 0;
            for (var i = 0; i < 4; i++) {
                s += this.elem(r,i)*B.elem(i,c);
            }
            AB.setElem(r,c,s);
        }
    }
    return AB;
};

Matrix4x4.prototype.concat = function(B) {
    var AB = this.mult(B);
    this.array = AB.array.slice(); // clone array
    return this;
};

Matrix4x4.prototype.scale = function(a, b, c) {
    var S = new Matrix4x4();
    S.set4x4(a, 0, 0, 0,
        0, b, 0, 0,
        0, 0, c, 0,
        0, 0, 0, 1);
    return this.concat(S);
};

Matrix4x4.prototype.translate = function(x, y, z) {
    var T = new Matrix4x4();
    T.set4x4(1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1);
    return this.concat(T);
};

// x^2(1-c)+c     xy(1-c)-zs     xz(1-c)+ys     0
// yx(1-c)+zs     y^2(1-c)+c     yz(1-c)-xs     0
// xz(1-c)-ys     yz(1-c)+xs     z^2(1-c)+c     0
// 0              0               0        1
Matrix4x4.prototype.rotate = function(angle_in_degrees, x, y, z) {
    var a = 1/Math.sqrt(x*x + y*y + z*z);
    x *= a;
    y *= a;
    z *= a;
    var theta = angle_in_degrees*Math.PI/180;
    var c = Math.cos(theta);
    var s = Math.sin(theta);
    var R = new Matrix4x4();
    R.set4x4(x*x*(1 - c) + c,   x*y*(1 - c) - z*s, x*z*(1 - c) + y*s, 0,
        y*x*(1 - c) + z*s, y*y*(1 - c) + c,   y*z*(1 - c) - x*s, 0,
        x*z*(1 - c) - y*s, y*z*(1 - c) + x*s, z*z*(1 - c) + c,   0,
        0,                 0,                 0,                 1);
    return this.concat(R);
};

//
// M <-- M * LookAt
//
Matrix4x4.prototype.lookAtTransform = function(eyex, eyey, eyez,
                                               lookatx, lookaty, lookatz,
                                               upx, upy, upz) {
    var eye = new Vec3(eyex, eyey, eyez);
    var lookat = new Vec3(lookatx, lookaty, lookatz);
    var Z = eye.sub(lookat).normalize();
    var u = new Vec3(upx, upy, upz);
    var X = u.cross(Z).normalize();
    var Y = Z.cross(X);

    var R = new Matrix4x4();
    R.set4x4(X.x,  X.y,  X.z, 0,
        Y.x,  Y.y,  Y.z, 0,
        -Z.x, -Z.y, -Z.z, 0,
        0,    0,    0, 1);
    R.translate(-eyex, -eyey, -eyez);
    return this.concat(R);
};

Matrix4x4.prototype.ortho = function(L, R,
                                     B, T,
                                     N, F) {
    var M = new Matrix4x4();
    M.set4x4(2/(R-L), 0,       0,       -(R+L)/(R-L),
        0,       2/(T-B), 0,       -(T+B)/(T-B),
        0,       0,      -2/(F-N), -(F+N)/(F-N),
        0,       0,       0,        1);
    return this.concat(M);
};

var Matrix4x4Stack = function() {
    this.stack = [];
};

Matrix4x4Stack.prototype.push = function(M) {
    var C = new Matrix4x4();
    C.copy(M);
    this.stack.push(C);
};

Matrix4x4Stack.prototype.pop = function(M) {
    var C = this.stack.pop();
    M.copy(C);
};
