
void receiveEvent(int bytes) {
  while(Wire.available())    // slave may send less than requested
  { 
    in_char = Wire.read(); // receive a byte as character 
    if((in_char!='i') && (in_char!='f')) 
    { 
      data[pos] = in_char;
      pos++;
    }
    //Serial.print("B: ");
    //Serial.println(data[pos]);   
  }
  data[pos]=0;
  pos=0;

//  Serial.println(data);
  parse_msg_velocities();
//  Serial.print("X=");Serial.print(vel_X,4);
//  Serial.print(" Y=");Serial.println(vel_Y,4);

  calcula_velocidade_motores();
}

void parse_msg_velocities() {
  char *token_from_msg;
  char delimiters[] = ",";
  uint8_t iterator = 0;
  //Serial.println(data);
  token_from_msg = strtok(data, delimiters); //This initializes strtok with our string to tokenize
  while (token_from_msg != NULL) {
    //Serial.println(token_from_msg);
    switch (iterator) {
      case 0:
        vel_X = atof (token_from_msg)/1000;
        break;
      case 1:
        vel_Y = atof (token_from_msg)/1000;
        break;
      case 2:
        vel_Wz = atof (token_from_msg)/1000;
        break;
      case 3:
        servo_ang = atof (token_from_msg);
        servoFlag = true;
        break;
    }
    
    token_from_msg = strtok(NULL, delimiters);    //Here we pass in a NULL value, which tells strtok to continue working with the previous string
    iterator++;
  }
}

void calcula_velocidade_motores()
{
  v1 = vel_X - vel_Y - ((C+L)/2)*vel_Wz;
  v2 = vel_X + vel_Y + ((C+L)/2)*vel_Wz;
  v3 = vel_X + vel_Y - ((C+L)/2)*vel_Wz;
  v4 = vel_X - vel_Y + ((C+L)/2)*vel_Wz;
}
