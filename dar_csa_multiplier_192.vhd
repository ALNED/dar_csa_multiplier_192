----------------------------------------------------------------------------
-- Double, Add and Reduce Multiplier using CSA (dar_csa_multiplier.vhd)
--
-- Multiply using the Double, Add and Reduce (DAR) Algorithm 
-- and the carry-stored encoding for intermediate values
--
----------------------------------------------------------------------------
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;
package dar_csa_multiplier_parameters is

   -- Parameters from Deschamps Book page 315

  constant k: integer := 192;   
  constant logk: integer := 8;  --logk is the number of bits of k-1
  constant m: std_logic_vector(k+1 downto 0) := "00" & X"fffffffffffffffffffffffffffffffeffffffffffffffff"; -- Decimal = 6277101735386680763835789423207666416083908700390324961279

  constant minus_m: std_logic_vector(k+1 downto 0) :="11" & X"000000000000000000000000000000010000000000000001"; --minus_m = 2**(k+2) - m

  --constant K: integer := 8;
  -- constant K: integer := 164; -- ou 164 ?  (10100011)
  --constant logK: integer := 3;  -- Log2(8) = 3
   -- constant logK: integer := 8;  -- Log2(163) = 7.34 ,the log function is the amount of bits that you need to represent the number. In this case 8
  --constant integer_M: integer := 239;
  --constant M: std_logic_vector(K+1 downto 0) := conv_std_logic_vector(integer_M, K+2); 
  
  -- constant M: std_logic_vector(K+1 downto 0):= "00" & x"4000000000000000000020108A2E0CC0D99F8A5EF"; --for M=163, copié du code K163 
  -- constant M: std_logic_vector(K+1 downto 0):= "0001000000000000000000000000000000000000000000000000000000000000000000000000000000001000000001000010001010001011100000110011000000110110011001111110001010010111101111" ;
 
 --constant minus_M: std_logic_vector(K+1 downto 0) := conv_std_logic_vector(2**(K+2) - integer_M, K+2);
  -- minus_M est préalablement calculé suivant la formule ci-dessus .
  --  2**(K+2) - integer_M 
  --  = 2^166 - x"4000000000000000000020108A2E0CC0D99F8A5EF" 
  --  = (93536104789177786765035829293842113257979682750464 - 5846006549323611672814741753598448348329118574063) base_10
  -- 	= (87690098239854175092221087540243664909650564176401) base_10
  --  = (3BFFFFFFFFFFFFFFFFFFFDFEF75D1F33F266075A11) base_Hex  sur K+2(166-bits si K=164).
  --  = ERROR : has 168 elements ; expected 166 elements.
  
  
   -- constant minus_M: std_logic_vector(K+1 downto 0):= "1110111111111111111111111111111111111111111111111111111111111111111111111111111111110111111110111101110101110100011111001100111111001001100110000001110101101000010001" ;
   -- constant minus_M: std_logic_vector(K+1 downto 0):=  x"3BFFFFFFFFFFFFFFFFFFFDFEF75D1F33F266075A11";
   --  constant minus_M: std_logic_vector(K+1 downto 0):= "00" & x"FFFFFFFFFFFFFFFFFFFFFFFF75D1F33F2660771F0" ;	  
   -- constant minus_M: std_logic_vector(K+1 downto 0):=  "111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111101110101110100011111001100111111001001100110000001110111000111110000";
 
    constant long_ZERO: std_logic_vector(K+1 downto 0) := (others => '0');
    constant ZERO: std_logic_vector(logK-1 downto 0) := (others => '0');
end dar_csa_multiplier_parameters;

----------------------------------------------------------------------------
--  dar_csa_multiplier
----------------------------------------------------------------------------
library ieee; 
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use work.dar_csa_multiplier_parameters.all;

entity dar_csa_multiplier is
port (
  x, y: in std_logic_vector(K-1 downto 0);
  clk, reset, start: in std_logic;
  z: out std_logic_vector(K-1 downto 0);
  done: out std_logic
);
end dar_csa_multiplier;

architecture rtl of dar_csa_multiplier is

  signal ps, next_ps, pc, p, u: std_logic_vector(k+1 downto 0) := (others=>'0'); 				-- Modif: Initialisation
  signal next_pc: std_logic_vector(k+1 downto 1) := (others=>'0'); 									-- Modif: Initialisation
  signal ss, sc, ws, wc, two_ps, two_pc, long_y: std_logic_vector(k+1 downto 0) := (others=>'0'); -- Modif: Initialisation
  signal int_x: std_logic_vector(k-1 downto 0) := (others=>'0');											  -- Modif: Initialisation

  signal quotient: std_logic_vector(1 downto 0) := (others=>'0'); 	 -- Modif: Initialisation
  signal t: std_logic_vector(2 downto 0) := (others=>'0');				 -- Modif: Initialisation

  signal step_type, equal_zero, condition, load, update, ce_p, x_i: std_logic := '0'; -- Modif: Initialisation

  type states is range 0 to 4;
  signal current_state: states;
  signal count: std_logic_vector(logk-1 downto 0) := (others=>'0'); 						   -- Modif: Initialisation

begin

  long_y <= "00" & y;
  sc(0) <= '0';
  --pc(0) <= '0';
  first_csa: for i in 0 to K generate
    ss(i) <= ps(i) xor pc(i) xor long_y(i);
    sc(i+1) <= (ps(i) and pc(i)) or (ps(i) and long_y(i)) or (pc(i) and long_y(i));
  end generate;
  ss(K+1) <= ps(K+1) xor pc(K+1) xor long_y(K+1);

  two_ps <= ps(K downto 0) & '0';
  two_pc <= pc(K downto 0) & '0';

  with step_type select ws <= two_ps when '0', ss when others;
  with step_type select wc <= two_pc when '0', sc when others;

  t <= ws(k+1 downto k-1) + wc(k+1 downto k-1);
  quotient(1) <= t(2) xor (t(1) and t(0)); 
  quotient(0) <= not(t(2) and t(1) and t(0));

  with quotient select u <= minus_M when "01", long_ZERO when "00", M when others;

 -- next_pc(0) <= '0';
  second_csa: for i in 0 to k generate
    next_ps(i) <= ws(i) xor wc(i) xor u(i);
    next_pc(i+1) <= (ws(i) and wc(i)) or (ws(i) and u(i)) or (wc(i) and u(i));
  end generate;
  next_ps(k+1) <= ws(k+1) xor wc(k+1) xor u(k+1);

  condition <= ce_p and (not(step_type) or x_i);

  parallel_register: process(clk)
  begin
  if clk'event and clk = '1' then
    if load = '1' then 
      ps <= (others => '0'); pc <= (others => '0');
    elsif condition = '1' then 
      ps <= next_ps; pc(k+1 downto 1) <= next_pc;
    end if;
  end if;
  end process parallel_register;

  equal_zero <= '1' when count = ZERO else '0';

  p <= ps + pc;

  with p(k) select z <= p(k-1 downto 0) when '0', 
                        p(k-1 downto 0) + m(k-1 downto 0) when others;

  shift_register: process(clk)
  begin
  if clk'event and clk='1' then
    if load = '1' then 
      int_x <= x;
    elsif update = '1' then
      for i in k-1 downto 1 loop int_x(i) <= int_x(i-1); end loop;
      int_x(0) <= '0';
    end if;
  end if;
  end process shift_register;

  x_i <= int_x(k-1);

  counter: process(clk)
  begin
  if clk'event and clk = '1' then
    if load = '1' then 
      count <= conv_std_logic_vector(K-1, logK);
    elsif update = '1' then 
      count <= count - 1;
    end if;
  end if;
  end process counter; 
   
  control_unit: process(clk, reset, current_state, equal_zero)
  begin
  case current_state is
    when 0 to 1 => step_type <= '0'; ce_p <= '0'; load <= '0'; update <= '0'; done <= '1';
    when 2 => step_type <= '0'; ce_p <= '0'; load <= '1'; update <= '0'; done <= '0';
    when 3 => step_type <= '0'; ce_p <= '1'; load <= '0'; update <= '0'; done <= '0';
    when 4 => step_type <= '1'; ce_p <= '1'; load <= '0'; update <= '1'; done <= '0';
  end case;

  if reset = '1' then 
    current_state <= 0;
  elsif clk'event and clk = '1' then
    case current_state is
      when 0 => if start = '0' then current_state <= 1; end if;
      when 1 => if start = '1' then current_state <= 2; end if;
      when 2 => current_state <= 3;
      when 3 => current_state <= 4;
      when 4 => if equal_zero = '1' then current_state <= 0; 
                else current_state <= 3; end if;
    end case;
  end if;
  end process control_unit;

end rtl;

