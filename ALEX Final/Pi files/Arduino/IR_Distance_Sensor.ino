void setupIR ()
{
  // Initialization sensor pin
  pinMode(LF_IR, INPUT);
  pinMode(LR_IR, INPUT);
  pinMode(RF_IR, INPUT);
  pinMode(RR_IR, INPUT);
  pinMode(RearIR, INPUT);
}

//in loop:
void proxyIRcheck()
{
  int LF_Val = digitalRead(LF_IR);
  int LR_Val = digitalRead(LR_IR);
  int RF_Val = digitalRead(RF_IR);
  int RR_Val = digitalRead(RR_IR);
  int Rear_Val = digitalRead(RearIR);

  //if 'HIGH', ok - No obstacle
  //if 'LOW', too close - Obstacle detected
  //ir_val |= (LF_Val << 12);
  //ir_val |= (RF_Val << 11);
  //ir_val |= (LR_Val << 10);
  //ir_val |= (RR_Val << 9);
  //ir_val |= (Rear_Val << 8);
  if (LF_Val == HIGH) {
    ir_val &= ~(1 << 13);
  } else {
    ir_val |= (1 << 13);
  }

  if (LR_Val == HIGH) {
    ir_val &= ~(1 << 11);
  } else {
    ir_val |= (1 << 11);
  }

  if (RF_Val == HIGH) {
    ir_val &= ~(1 << 12);
  } else {
    ir_val |= (1 << 12);
  }

  if (RR_Val == HIGH) {
    ir_val &= ~(1 << 10);
  } else {
    ir_val |= (1 << 10);
  }

  if (Rear_Val == HIGH) {
    ir_val &= ~(1 << 9);
  } else { 
    ir_val |= (1 << 9);
  }
  /*if (LF_Val == HIGH) {
    LF_IR_Val = 1;
    } else {
    LF_IR_Val = 0;
    }

    if (LR_Val == HIGH) {
    LR_IR_Val = 1;
    } else {
    LR_IR_Val = 0;
    }

    if (RF_Val == HIGH) {
    RF_IR_Val = 1;
    } else {
    RF_IR_Val = 0;
    }

    if (RR_Val == HIGH) {
    RF_IR_Val = 1;
    } else {
    RF_IR_Val = 0;
    }

    if (Rear_Val == HIGH) {
    Rear_IR_Val = 1;
    } else {
    Rear_IR_Val = 0;
    }

  */
}
