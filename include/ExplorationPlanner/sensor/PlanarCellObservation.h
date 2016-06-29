#ifndef PLANARCELLOBSERVATION_H
#define PLANARCELLOBSERVATION_H

class PlanarCellObservation
{
public:
    PlanarCellObservation()
        : o_(false), inverse_sensor_model_(0.0)
    {
    }

    PlanarCellObservation(bool o, double ism)
        : o_(o), inverse_sensor_model_(ism)
    {
    }

    bool getObservation() const {
        return o_;
    }

    double getISMValue() const {
        return inverse_sensor_model_;
    }

private:
    bool o_;
    double inverse_sensor_model_;
};

#endif // PLANARCELLOBSERVATION_H
