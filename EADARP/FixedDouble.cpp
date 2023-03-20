#include "FixedDouble.h"


FixedDouble::FixedDouble() : value_(0.0), decimal_places_(0) {}
FixedDouble::FixedDouble(double value, int decimal_places) : value_(round(value* pow(10.0, decimal_places)) / pow(10.0, decimal_places)), decimal_places_(decimal_places) {}


FixedDouble::FixedDouble(const FixedDouble& other) : value_(other.value_), decimal_places_(other.decimal_places_) {}

FixedDouble& FixedDouble::operator=(const FixedDouble& other) {
    if (this != &other) {
        value_ = other.value_;
        decimal_places_ = other.decimal_places_;
    }
    return *this;
}

FixedDouble FixedDouble::operator+(const FixedDouble& other) const {
    int max_dp = std::max(decimal_places_, other.decimal_places_);
    return FixedDouble(value_ + other.value_, max_dp);
}


FixedDouble FixedDouble::operator-(const FixedDouble& other) const {
    int max_dp = std::max(decimal_places_, other.decimal_places_);
    return FixedDouble(value_ - other.value_, max_dp);
}

FixedDouble FixedDouble::operator*(const FixedDouble& other) const {
    return FixedDouble(value_ * other.value_, decimal_places_ + other.decimal_places_);
}


FixedDouble FixedDouble::operator/(const FixedDouble& other) const {
    return FixedDouble(value_ / other.value_, decimal_places_ - other.decimal_places_);
}


std::ostream& operator<<(std::ostream& os, const FixedDouble& fixed_double) {
    os << std::fixed << std::setprecision(fixed_double.decimal_places_) << fixed_double.value_;
    return os;
}

bool FixedDouble::operator==(const FixedDouble& other) const {
    double epsilon = pow(10.0, -decimal_places_);
    return std::abs(value_ - other.value_) < epsilon;
}

bool FixedDouble::operator<(const FixedDouble& other) const {
    if (decimal_places_ == other.decimal_places_) {
        return value_ < other.value_;
    }
    else {
        FixedDouble lhs = FixedDouble(value_, other.decimal_places_);
        FixedDouble rhs = FixedDouble(other.value_, decimal_places_);
        return lhs.value_ < rhs.value_;
    }
}

double FixedDouble::getValue() const
{
    return this->value_;
}

int FixedDouble::getDigits() const
{
    return this->decimal_places_;
}

FixedDouble min(const FixedDouble& left, const FixedDouble& right) {
    if (left < right) {
        return left;
    }
    else {
        return right;
    }
}

FixedDouble max(const FixedDouble& left, const FixedDouble& right) {
    if (left < right) {
        return right;
    }
    else {
        return left;
    }
}
