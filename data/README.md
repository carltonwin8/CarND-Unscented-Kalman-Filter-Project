# UKF Data Analysis

The repo for the code that processed the file below is on
[github](https://github.com/carltonwin8/CarND-Mercedes-SF-Utilities).

| File | Description |
| --- | --- |
| dataset1.txt | ukf filter input from the simulator |

## dataset1.txt

The input to the ukf from the simulator that is generate via the C++
noted below. The top 2 lines are removed with vi before is it is
processed with [ukf-input.ipynb]().

```cpp
if (m.sensor_type_ == MeasurementPackage::LASER) {
  cout << "L "
    << m.raw_measurements_(0) << " "
    << m.raw_measurements_(1) << " "
    << "0" << " "
    << m.timestamp_ << " ";
} else {
  cout << "R "
    << m.raw_measurements_(0) << " "
    << m.raw_measurements_(1) << " "
    << m.raw_measurements_(2) << " "
    << m.timestamp_ << " ";
}
```
