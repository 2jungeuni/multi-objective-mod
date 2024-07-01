# multi-objective-mod

### Compile
```bash
c++ -O3 -Wall -shared -std=c++11 -fPIC $(python3 -m pybind11 --includes) astar.h astar.cpp planner.cpp -o planner.so
```

### :runner: Run demo code
If you want to get the optimal cost,
```python
python3 main.py -v ${number_of_vehicles} -w ${alpha} ${beta} ${gamma} -pudo ${pickup_location_1} ${dropoff_location_1} -pudo ${pickup_location_2} ${dropoff_location_2}
```
For example,
```python3
python3 main.py -v 3 -w 1 0 0 -pudo 47 52 -pudo 244 642 -pudo 508 1572 -pudo 60 342 -pudo 440 1014
```