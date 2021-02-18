

class Ranging {
    public:
    const uint64_t SPEED_OF_LIGHT = 299792458L; 
    DW1000Time PollTxTime;
    DW1000Time RespRxTime;
    DW1000Time FinalTxTime;


    DW1000Time PollRxTime_T;
    DW1000Time RespRxTime_T;
    DW1000Time FinalRxTime_T;
    
    DW1000Time Ra;
    DW1000Time Rb;
    DW1000Time Da;
    DW1000Time Db;

    DW1000Time prev_Db;
    DW1000Time prev_Rb;

    DW1000Time RT1;
    DW1000Time RT2;
   
    void calculate_TIME(){
              //print_all_time();

        Ra = (RespRxTime - PollTxTime).wrap();
        Da = (FinalTxTime - RespRxTime).wrap();

        RT1 = (RespRxTime_T-PollRxTime_T).wrap();
        RT2 = (FinalRxTime_T-RespRxTime_T).wrap();
      }

    double calculateTDoARange() {

        //print_all_time();
        DW1000Time TDoA;
        TDoA = (RT1*prev_Rb-prev_Db*RT2+Da*RT1-RT2*Ra)/2/(RT1+RT2);
        //Serial.print("ToF: ");
        //Serial.println(ToF);
        float TDoA_microseconds = TDoA.getAsMicroSeconds();
        double output = (double)(TDoA_microseconds*1000);
        Serial.print("Output: ");
        
        Serial.println(output);
        return (int32_t)(TDoA_microseconds * SPEED_OF_LIGHT);
        
    }

    void initialize() {
      PollTxTime.setTimestamp((int64_t)0);
      PollRxTime_T.setTimestamp((int64_t)0);
      RespRxTime_T.setTimestamp((int64_t)0);
      RespRxTime.setTimestamp((int64_t)0);
      FinalTxTime.setTimestamp((int64_t)0);
      FinalRxTime_T.setTimestamp((int64_t)0);
    }


    void print_all_time(){
      Serial.print("Ra: ");
      Serial.println(Ra.getAsMicroSeconds());
      Serial.print("Db: ");
      Serial.println(prev_Db.getAsMicroSeconds());
      Serial.print("RT1: ");
      Serial.println(RT1.getAsMicroSeconds());      
      
      Serial.print("Rb: ");
      Serial.println(prev_Rb.getAsMicroSeconds());      
      Serial.print("Da: ");
      Serial.println(Da.getAsMicroSeconds());
      Serial.print("RT2: ");
      Serial.println(RT2.getAsMicroSeconds());   
      }

      
//    void printAll() {
//      Serial.print("PollTxTime: ");
//      Serial.println(PollTxTime);
//      Serial.print("RespRxTime: ");
//      Serial.println(RespRxTime);
//
//      Serial.print("RespTxTime: ");
//      Serial.println(RespTxTime);
//      Serial.print("FinalRxTime: ");
//      Serial.println(FinalRxTime);
//
//      Serial.print("FinalTxTime: ");
//      Serial.println(FinalTxTime);
//      Serial.print("PollRxTime: ");
//      Serial.println(PollRxTime);
//
//      
//    }
    
};
