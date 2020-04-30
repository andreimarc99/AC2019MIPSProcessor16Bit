library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity IFetch is
    Port (clk: in STD_LOGIC;
          rst : in STD_LOGIC;
          en : in STD_LOGIC;
          BranchAddress : in STD_LOGIC_VECTOR(15 downto 0);
          JumpAddress : in STD_LOGIC_VECTOR(15 downto 0);
          Jump : in STD_LOGIC;
          PCSrc : in STD_LOGIC;
          Instruction : out STD_LOGIC_VECTOR(15 downto 0);
          PCinc : out STD_LOGIC_VECTOR(15 downto 0));
end IFetch;

architecture Behavioral of IFetch is

-- Memorie ROM
type tROM is array (0 to 255) of STD_LOGIC_VECTOR (15 downto 0);
signal ROM : tROM := (

-- PROGRAM DE TEST
-- Acest program testeaza toate instructiunile implementate,
-- folosind scrierea si citirea din memorie pentru verificare
-- si, de asemenea, instructiunile de salt BEQ si J.
--    B"000_001_000_010_0_000",   -- X"0420" -- ADD $2, $1, $0 
--    B"000_011_010_010_0_001",   -- X"0d21" -- SUB $2, $3, $2
--    B"000_000_010_010_1_010",   -- X"012A" -- SLL $2, $2, 1
--    B"000_000_010_010_1_011",   -- X"012b" -- SRL $2, $2, 1
--    B"000_011_010_100_0_100",   -- X"0d44" -- AND $4, $3, $2
--    B"000_101_100_100_0_101",   -- X"1645" -- OR $4, $5, $4
--    B"000_100_100_100_0_110",   -- X"1246" -- XOR $4, $4, $4
--    B"000_010_011_100_0_111",   -- X"09C7" -- SLT $4, $2, $3
--    B"001_000_100_0000100",     -- X"2204" -- ADDI $4, $0, 4
--    B"010_001_101_0000000",     -- X"4680" -- LW $5, 0($1)
--    B"011_100_101_0000000",     -- X"7280" -- SW $5, 0($4)
--    B"100_001_001_0000001",     -- X"8481" -- BEQ $1, $1, 1
--    B"101_100_101_0000100",     -- X"b284" -- ANDI $5, $4, 4
--    B"110_101_110_0000011",     -- X"d703" -- ORI $6, $5, 3
--    B"111_0000000000011",       -- X"E003" -- J 3

-- FIBONACCI		
-- Acest program calculeaza sirul lui Fibonacci
-- incarcand initial 0 si 1 in 2 registri.
-- Se efectueaza scrierea in memorie la 2 adrese diferite
-- si apoi citirea de la aceleasi adrese pentru a verifica 
-- corectitudinea. Calculul elementelor din sir se face 
-- intr-o bucla, folosind instructiunea J.
    --B"001_000_001_0000000",     -- X"2080" -- ADDI $1, $0, 0
    --B"001_000_010_0000001",     -- X"2101" -- ADDI $2, $0, 1	
    --B"001_000_011_0000000",     -- X"2180" -- ADDI $3, $0, 0	
    --B"001_000_100_0000001",     -- X"2201" -- ADDI $4, $0, 1
    --B"011_011_001_0000000",     -- X"6C80" -- SW $1, 0($3)
    --B"011_100_010_0000000",     -- X"7100" -- SW $2, 0($4)
    --B"010_011_001_0000000",     -- X"4C80" -- LW $1, 0($3)
    --B"010_100_010_0000000",     -- X"5100" -- LW $2, 0($4)
    --B"000_001_010_101_0_000",   -- X"0550" -- ADD $5, $1, $2
    --B"000_000_010_001_0_000",   -- X"0110" -- ADD $1, $0, $2
    --B"000_000_101_010_0_000",   -- X"02A0" -- ADD $2, $0, $5
    --B"111_0000000001000",       -- X"E008" -- J 8
    
    X"2800", -- ADDI $1, $0, 0
    X"2805", -- ADDI $2, $0, 5
    X"2C00", -- ADDI $3, $0, 0
    X"0000", -- NOOP
    X"8506", -- BEQ $1, $2, 6
    X"0000", -- NOOP
    X"0000", -- NOOP
    X"0000", -- NOOP
    X"0CB0", -- ADD $3, $3, $1
    X"2481", -- ADDI $1, $1, 1
    X"E004", -- J 4
    X"0000", -- NOOP
    X"6580", -- SW $3, 0($1)

    others => X"0000");

signal PC : STD_LOGIC_VECTOR(15 downto 0) := (others => '0');
signal PCAux, NextAddr, AuxSgn, AuxSgn1: STD_LOGIC_VECTOR(15 downto 0);

begin

    -- Program Counter
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                PC <= (others => '0');
            elsif en = '1' then
                PC <= NextAddr;
            end if;
        end if;
    end process;

    -- Instruction OUT
    Instruction <= ROM(conv_integer(PC(7 downto 0)));

    -- PC incremented
    PCAux <= PC + 1;
    PCinc <= PCAux;

    -- MUX Branch
    process(PCSrc, PCAux, BranchAddress)
    begin
        case PCSrc is 
            when '1' => AuxSgn <= BranchAddress;
            when others => AuxSgn <= PCAux;
        end case;
    end process;	

     -- MUX Jump
    process(Jump, AuxSgn, JumpAddress)
    begin
        case Jump is
            when '1' => NextAddr <= JumpAddress;
            when others => NextAddr <= AuxSgn;
        end case;
    end process;

end Behavioral;