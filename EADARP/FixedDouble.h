#pragma once
#include <iostream>
#include <iomanip>
#include <cmath>

class FixedDouble {
private:
    double value_;
    int decimal_places_;

public:
    // Constructors
    FixedDouble();
    FixedDouble(double value, int decimal_places);

    // Copy constructor
    FixedDouble(const FixedDouble& other);

    // Assignment operator
    FixedDouble& operator=(const FixedDouble& other);

    // Addition operator
    FixedDouble operator+(const FixedDouble& other) const;

    // Subtraction operator
    FixedDouble operator-(const FixedDouble& other) const;

    // Multiplication operator
    FixedDouble operator*(const FixedDouble& other) const;

    // Division operator
    FixedDouble operator/(const FixedDouble& other) const;

    // Output stream operator
    friend std::ostream& operator<<(std::ostream& os, const FixedDouble& fixed_double);

    //Equals operator
    bool operator==(const FixedDouble& other) const;

    FixedDouble min(const FixedDouble& left, const FixedDouble& right);

    FixedDouble max(const FixedDouble& left, const FixedDouble& right) {
        if (left < right) {
            return right;
        }
        else {
            return left;
        }
    }

    bool operator<(const FixedDouble& other) const;

    double getValue() const;
    int getDigits() const;


};