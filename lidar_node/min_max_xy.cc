#include "min_max_xy.hh"
#include <cmath>

float roundToMm(float value)
{
    return std::round(value * 1000.0f) / 1000.0f;
}

bool MinMaxXY::update(float x, float y)
{
    bool changed = false;

    x = roundToMm(x);
    y = roundToMm(y);

    if (!minX.has_value() || x < minX.value())
    {
        minX = x;
        changed = true;
    }

    if (!maxX.has_value() || x > maxX.value())
    {
        maxX = x;
        changed = true;
    }

    if (!minY.has_value() || y < minY.value())
    {
        minY = y;
        changed = true;
    }

    if (!maxY.has_value() || y > maxY.value())
    {
        maxY = y;
        changed = true;
    }

    return changed;
}

std::optional<float> MinMaxXY::getMinX() const
{
    return minX;
}

std::optional<float> MinMaxXY::getMaxX() const
{
    return maxX;
}

std::optional<float> MinMaxXY::getMinY() const
{
    return minY;
}

std::optional<float> MinMaxXY::getMaxY() const
{
    return maxY;
}

void MinMaxXY::setMinX(float value)
{
    minX = value;
}

void MinMaxXY::setMaxX(float value)
{
    maxX = value;
}

void MinMaxXY::setMinY(float value)
{
    minY = value;
}

void MinMaxXY::setMaxY(float value)
{
    maxY = value;
}

void MinMaxXY::reset()
{
    minX = std::nullopt;
    maxX = std::nullopt;
    minY = std::nullopt;
    maxY = std::nullopt;
}