#pragma once

#include <frc/util/Color.h>

namespace rev {

class CIEColor {
public:    
    CIEColor(double X, double Y, double Z) : X(X), Y(Y), Z(Z) {}

    double X;
    double Y;
    double Z;
private:
    // This is private until we get it working correctly
    frc::Color ToRGB();
    static const double IlluminantD65[3];
    static const double XYZtoRGB[9];
};

} // namespace rev
