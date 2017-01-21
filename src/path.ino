// void findAvaliblePath() {
//   if (frontCenterSensorState) {
//     return 1;//forward
//   }else if (frontRightSensorState) {
//     if (sideRightSensorState) {
//       return 3;//turn Right
//     }
//     return 2;//right diagonal
//   }else if (frontLeftSensorState) {
//     if (sideLeftSensorState) {
//       return 5;//turn left
//     }
//     return 4;//left diagonal
//   } else if (rearLeftSensorState){
//     return 6;//turn around and then turn left
//   } else if (rearRightSensorState){
//     return 7;//turn around and then turn Right
//   }else if (rearCenterSensorState){
//     return 8;//turn right and if a side is becomes open turn that way
//   }
//
// }

void exPath(){
  int pathIndex = findAvaliblePath();
  switch (pathIndex) {
    case 1:
      forwardAuto(fullSpeed);
    break;
  }
}
void forwardAuto(int speed){

}
