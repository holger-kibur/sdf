...
CombLogic: PROCESS (CurrState, C1, C0, Rst)
  BEGIN
    CASE CurrState IS
      WHEN S_Off =>
        IF (C1 = '0') THEN
          NextState <= S_Off;
        ELSE
          IF (C0 = '1' AND Rst = '1') THEN
            NextState <= S_On2;
          ELSE
            NextState <= S_On1;
          END IF;
        END IF;
      WHEN S_On1 =>
         IF (C0 = '0') THEN
           NextState <= S_On1;
         ELSE
           NextState <= S_On2;
         END IF;
       WHEN S_On1 =>
         IF (C1 = '0') THEN
           NextState <= S_On2;
         ELSE
           NextState <= S_On1;
         END IF;
       END CASE;
END PROCESS CombLogic;

StateReg: PROCESS (Clk, Rst)
BEGIN
  IF (Clk = '1' AND Clk'EVENT) THEN
    IF (Rst = '1') THEN
      CurrState <= S_Off;
    ELSE
      CurrState <= NextState;
    END IF;
  END IF;
END PROCESS StateReg;
...