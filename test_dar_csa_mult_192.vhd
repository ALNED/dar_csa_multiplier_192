--------------------------------------------------------------------------------
-- 
-- VHDL Test Bench for module: 
--
-- Executes an exhaustive Test Bench.
--
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
--USE IEEE.std_logic_arith.all;
USE ieee.std_logic_signed.all;
USE ieee.std_logic_unsigned.all;
USE ieee.numeric_std.ALL;
USE ieee.std_logic_textio.ALL;
USE std.textio.ALL;
--use work.csa_mod_multiplier_parameters.all;
use work.dar_csa_multiplier_parameters.all;   --  K and M are constants in my_package


-- k: integer := 192; 
-- m: std_logic_vector(k+1 downto 0) := "00" & X"fffffffffffffffffffffffffffffffeffffffffffffffff";


ENTITY test_dar_csa_mult IS
END test_dar_csa_mult;

ARCHITECTURE behavior OF test_dar_csa_mult IS 

  -- Component Declaration for the Unit Under Test (UUT)
  COMPONENT dar_csa_multiplier is
  PORT (
    x, y: in std_logic_vector(K-1 downto 0);  
    clk, reset, start: in std_logic; 
    z: out std_logic_vector(K-1 downto 0);  
    done: inout std_logic
    );
  END COMPONENT;
  --Inputs
  SIGNAL x, y :  std_logic_vector(K-1 downto 0) := (others=>'0');  
  SIGNAL clk, reset, start, done: std_logic := '0';  -- Modif : Initialisation
  --Outputs
  SIGNAL z :  std_logic_vector(K-1 downto 0) := (others=>'0') ; 

  constant DELAY : time := 100 ns;  
  constant PERIOD : time := 200 ns; 
  constant DUTY_CYCLE : real := 0.5;
  constant OFFSET : time := 0 ns;


BEGIN

  -- Instantiate the Unit Under Test (UUT)
  uut: dar_csa_multiplier PORT MAP(x => x, y => y,   				-- Modification : suppression de m
                          clk => clk, reset => reset, start => start,
                          z => z, done => done);
                          
   PROCESS -- clock process for clk
   BEGIN
    WAIT for OFFSET;
    CLOCK_LOOP : LOOP
       clk <= '0';
       WAIT FOR (PERIOD *(1.0 - DUTY_CYCLE));
       clk <= '1';
       WAIT FOR (PERIOD * DUTY_CYCLE);
    END LOOP CLOCK_LOOP;
   END PROCESS;

  tb_proc : PROCESS --generate values
   --  VARIABLE m_int: natural;         -- commenté
    VARIABLE TX_LOC : LINE;
    VARIABLE TX_STR : String(1 to 4096);

	 
  Variable result: std_logic_vector(K-1 downto 0) := (others=>'0');   -- adapté pour recevoir le résultat de l'opération sur 163-bit. K est déclaré dans my_package.all
  
  Variable u_result: unsigned(K-1 downto 0) := (others=>'0') ; -- Conversion pour faire la comparaison, 
  Variable u_z: unsigned(K-1 downto 0) := (others=>'0') ; 	  -- Usage similar to STD_LOGIC_VECTOR   
	 
		constant const1_192_bit: std_logic_vector(K-1 downto 0):= x"3000000000000000000010108A2E0AB0D9221A5EF0000000"; 
		constant const2_192_bit: std_logic_vector(K-1 downto 0):= x"2000000000000000000010307B2E0AB0D6221C4FE0000000";
    constant const_mult_192_bit: std_logic_vector(K-1 downto 0):= x"A555C563A8119654A76BEA8B5904B36D6B4AA89AD9041377"; -- Hamza soft : keygener assist
  -- const_mult_163_bit : is the reminder of the Multiplication of const1 and const2 by M.
	 
	 -- const1_163_bit * const2_163_bit = "6000000000000000000050B285E6357434AA89AD90413779F55C0587FB332FD5E213CE557A23C9F22"
	 
    BEGIN
      start <= '0'; reset <= '1';
      WAIT FOR PERIOD;
      reset <= '0';
      WAIT FOR PERIOD;
		
     --m_int := 239;							 -- commenté, c'est l'équivalent de integer_M	  (Dans CSA Package) 
     --m <= CONV_STD_LOGIC_VECTOR (m_int, 8);    -- commenté, c'est l'équivalent de M   		  (Dans CSA Package) 
      
--		for J in 0 to 238 loop
--        for I in 0 to 238 loop
--          x <= CONV_STD_LOGIC_VECTOR (I, 8); 
--          y <= CONV_STD_LOGIC_VECTOR (J, 8);
--			 WAIT FOR PERIOD; -- Modif : Donner du temps pour que la ligne "when 0 => if start = '0' then current_state <= 1;" s'exécute à temps
--          start <= '1'; 
--          WAIT FOR PERIOD;
--          start <= '0'; 
--          wait until done = '1'; --Pb !!
--          WAIT FOR PERIOD;
--          IF ( ((I*J) mod integer_M) /= ieee.std_logic_unsigned.CONV_INTEGER(z) ) THEN  -- Modification : m_int --> integer_M
--            write(TX_LOC,string'("ERROR!!! X=")); write(TX_LOC, x);
--            write(TX_LOC,string'("* Y=")); write(TX_LOC, y);
--            write(TX_LOC,string'(" mod M=")); write(TX_LOC, M);                 -- Modification : m --> M
--            write(TX_LOC,string'(" is Z=")); write(TX_LOC, z);
--            write(TX_LOC,string'(" instead of:")); write(TX_LOC, (I*J) mod integer_M);   -- Modification  : m_int --> integer_M
--            write(TX_LOC, string'(" "));
--            write(TX_LOC,string'(" (i=")); write(TX_LOC, i);
--            write(TX_LOC,string'(" j=")); write(TX_LOC, j); 
--            write(TX_LOC, string'(")"));
--            TX_STR(TX_LOC.all'range) := TX_LOC.all;
--            Deallocate(TX_LOC);
--            ASSERT (FALSE) REPORT TX_STR SEVERITY ERROR;
--          END IF;  
--          end loop;
--      end loop;



	-- Conversion des types "STD_LOGIC_VECTOR" à "Unsigned" (Bibliothèque ieee.std_logic_unsigned) pour la comparaison.

  
	-- Les INPUTS du UUT
   	   
			x <= const1_192_bit ;  -- affectation de l'opérande X.
			y <= const2_192_bit ; -- affectation de l'opérande y.
	
			 WAIT FOR PERIOD; -- Modif : Donner du temps pour que la ligne "when 0 => if start = '0' then current_state <= 1;" s'exécute à temps
          start <= '1'; 
          WAIT FOR PERIOD;
          start <= '0'; 
          wait until done = '1'; --Pb !!
          WAIT FOR PERIOD;
	
			result := const_mult_192_bit;   -- résultat de la multiplication modulo M , précalculé avec "Keygener assistant".
				          		   							
				 u_z := unsigned(z(K-1 downto 0)) ;			 
				 u_result := unsigned(result(K-1 downto 0)) ;
				 
				--IF ( result /= ieee.std_logic_unsigned.CONV_INTEGER(z) ) THEN 
				
				IF ( u_z /= u_result ) THEN 
			 
			   write(TX_LOC,string'("ERROR! 'Z = X * Y mod M' does not match expected Z. CR")); --write(TX_LOC, x);
            write(TX_LOC,string'("X=")); write(TX_LOC, x);
            write(TX_LOC,string'("Y=")); write(TX_LOC, y);
            write(TX_LOC,string'("M=")); write(TX_LOC, M);                 -- Modification : m --> M
            write(TX_LOC,string'("Z=")); write(TX_LOC, z);
            write(TX_LOC,string'("Expected Z =")); write(TX_LOC, result);   -- Modification  : result
            write(TX_LOC, string'(" "));
--            write(TX_LOC,string'(" (i=")); write(TX_LOC, i);
--            write(TX_LOC,string'(" j=")); write(TX_LOC, j); 
            write(TX_LOC, string'(")"));
            TX_STR(TX_LOC.all'range) := TX_LOC.all;
            Deallocate(TX_LOC);
            ASSERT (FALSE) REPORT TX_STR SEVERITY ERROR;
          END IF;  
	

    WAIT FOR DELAY;
    ASSERT (FALSE) REPORT
    "Simulation successful (not a failure).  No problems detected. "
    SEVERITY FAILURE;
   END PROCESS;

END;
