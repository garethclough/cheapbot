#ifndef MIN_MAX_XY_HH
#define MIN_MAX_XY_HH

#include <optional>

class MinMaxXY
{
private:
    std::optional<float> minX;
    std::optional<float> maxX;
    std::optional<float> minY;
    std::optional<float> maxY;

public:
    bool update(float x, float y);

    std::optional<float> getMinX() const;
    std::optional<float> getMaxX() const;
    std::optional<float> getMinY() const;
    std::optional<float> getMaxY() const;

    void setMinX(float value);
    void setMaxX(float value);
    void setMinY(float value);
    void setMaxY(float value);

    void reset();
};

#endif