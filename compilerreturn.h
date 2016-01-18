// read on arduino site if you want to return a type,
// you have to put it in an include. likewise for the class.

// direction enum
typedef enum Direction{
north, east, south, west
};

// used for previous terms
typedef enum Turn{
left, right
};

// this just a helper class to hold all the information together
// for the mapping
class DirectionInfo{
  public:
  Direction leadDirection;
  Direction offDirection;
  float leadAngleDifference;
};
