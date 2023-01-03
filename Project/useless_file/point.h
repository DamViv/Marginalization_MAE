#pragma once

class Point {
   public:
    Point();
    Point(float x, float y);
    virtual ~Point();

    float mX;
    float mY;
};
