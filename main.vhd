library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity main is
    Port ( clk : in STD_LOGIC;
           btn : in STD_LOGIC_VECTOR (4 downto 0);
           sw : in STD_LOGIC_VECTOR (15 downto 0);
           led : out STD_LOGIC_VECTOR (15 downto 0);
           an : out STD_LOGIC_VECTOR (3 downto 0);
           cat : out STD_LOGIC_VECTOR (6 downto 0);
           rx : in STD_LOGIC;
           tx : out STD_LOGIC );
end main;

architecture Behavioral of main is

component FSM is
port (
    clk, rst, tx_en, baud_en : IN std_logic;
    tx_data : IN std_logic_vector(7 downto 0);
    tx, tx_rdy : OUT std_logic
);
end component;

component MPG is
    Port ( en : out STD_LOGIC;
           input : in STD_LOGIC;
           clock : in STD_LOGIC);
end component;

component SSD is
    Port ( clk: in STD_LOGIC;
           digits: in STD_LOGIC_VECTOR(15 downto 0);
           an: out STD_LOGIC_VECTOR(3 downto 0);
           cat: out STD_LOGIC_VECTOR(6 downto 0));
end component;

component IFetch
    Port ( clk: in STD_LOGIC;
           rst : in STD_LOGIC;
           en : in STD_LOGIC;
           BranchAddress : in STD_LOGIC_VECTOR(15 downto 0);
           JumpAddress : in STD_LOGIC_VECTOR(15 downto 0);
           Jump : in STD_LOGIC;
           PCSrc : in STD_LOGIC;
           Instruction : out STD_LOGIC_VECTOR(15 downto 0);
           PCinc : out STD_LOGIC_VECTOR(15 downto 0));
end component;

component IDecode
    Port ( clk: in STD_LOGIC;
           en : in STD_LOGIC;    
           Instr : in STD_LOGIC_VECTOR(12 downto 0);
           WD : in STD_LOGIC_VECTOR(15 downto 0);
           RegWrite : in STD_LOGIC;
           RegDst : in STD_LOGIC;
           ExtOp : in STD_LOGIC;
           RD1 : out STD_LOGIC_VECTOR(15 downto 0);
           RD2 : out STD_LOGIC_VECTOR(15 downto 0);
           Ext_Imm : out STD_LOGIC_VECTOR(15 downto 0);
           func : out STD_LOGIC_VECTOR(2 downto 0);
           sa : out STD_LOGIC);
end component;

component MainControl
    Port ( Instr : in STD_LOGIC_VECTOR(2 downto 0);
           RegDst : out STD_LOGIC;
           ExtOp : out STD_LOGIC;
           ALUSrc : out STD_LOGIC;
           Branch : out STD_LOGIC;
           Jump : out STD_LOGIC;
           ALUOp : out STD_LOGIC_VECTOR(2 downto 0);
           MemWrite : out STD_LOGIC;
           MemtoReg : out STD_LOGIC;
           RegWrite : out STD_LOGIC);
end component;

component ExecutionUnit is
    Port (
           RD1 : in STD_LOGIC_VECTOR(15 downto 0);
           RD2 : in STD_LOGIC_VECTOR(15 downto 0);
           Ext_Imm : in STD_LOGIC_VECTOR(15 downto 0);
           func : in STD_LOGIC_VECTOR(2 downto 0);
           sa : in STD_LOGIC;
           ALUSrc : in STD_LOGIC;
           ALUOp : in STD_LOGIC_VECTOR(2 downto 0);
           ALURes : out STD_LOGIC_VECTOR(15 downto 0);
           Zero : out STD_LOGIC);
end component;

component MEM
    port ( clk : in STD_LOGIC;
           en : in STD_LOGIC;
           ALUResIn : in STD_LOGIC_VECTOR(15 downto 0);
           RD2 : in STD_LOGIC_VECTOR(15 downto 0);
           MemWrite : in STD_LOGIC;			
           MemData : out STD_LOGIC_VECTOR(15 downto 0);
           ALUResOut : out STD_LOGIC_VECTOR(15 downto 0));
end component;

signal Instruction, PCinc, RD1, RD2, WD, Ext_imm : STD_LOGIC_VECTOR(15 downto 0); 
signal JumpAddress, BranchAddress, ALURes, ALURes1, MemData : STD_LOGIC_VECTOR(15 downto 0);
signal func : STD_LOGIC_VECTOR(2 downto 0);
signal sa, zero : STD_LOGIC;
signal digits : STD_LOGIC_VECTOR(15 downto 0);
signal en, rst, PCSrc : STD_LOGIC; 
-- main controls 
signal RegDst, ExtOp, ALUSrc, Branch, Jump, MemWrite, MemtoReg, RegWrite : STD_LOGIC;
signal ALUOp :  STD_LOGIC_VECTOR(2 downto 0);

signal reg_mux : std_logic_vector(2 downto 0);
signal mem_mux : std_logic_vector(15 downto 0);

-- pipe registers
signal if_id  : std_logic_vector(31 downto 0);
signal id_ex  : std_logic_vector(81 downto 0);
signal ex_mem : std_logic_vector(55 downto 0);
signal mem_wb : std_logic_vector(36 downto 0);

signal fsm_cnt : std_logic_vector(13 downto 0);
signal baud_enable : std_logic;
signal mpg_out1 : std_logic;
signal mpg_out2 : std_logic;
signal q_bist : std_logic;

begin

    fsm_counter : process(clk)
    begin
        if (rising_edge(clk)) then
            fsm_cnt <= fsm_cnt + 1;
            if fsm_cnt = 10415 then 
                baud_enable <= '1';
            else 
                baud_enable <= '0';
            end if;
        end if;
    end process;
    
    bist: process(clk, mpg_out2, baud_enable)
    begin
        if baud_enable = '1' then 
            q_bist <= '0';
        else if rising_edge(clk) then
                q_bist <= mpg_out2;
            end if;
        end if; 
    end process;

    MPG1 : MPG port map(mpg_out1, btn(2), clk);
    MPG2 : MPG port map(mpg_out2, btn(3), clk);
    FSM1 : FSM port map(clk, mpg_out1, q_bist, baud_enable, sw(7 downto 0), TX, led(15));
    display : SSD port map (clk, digits, an, cat);
    
    
    -- pipe registers processes 
    --IF/ID
    process(clk, en, PCinc, Instruction) 
    begin
        if rising_edge(clk) then
            if en = '1' then
                if_id(31 downto 16) <= PCinc;
                if_id(15 downto 0) <= Instruction;
            end if;
        end if;
    end process;
    
    --ID/EX
    process(if_id, MemToReg, RegWrite, MemWrite, Branch, ALUOp, ALUSrc, RegDst, clk, en, Ext_Imm)
    begin
        if rising_edge(clk) then
            if en = '1' then
                id_ex(81) <= MemToReg;
                id_ex(80) <= RegWrite;
                id_ex(79) <= MemWrite;
                id_ex(78) <= Branch;
                id_ex(77 downto 75) <= ALUOp;
                id_ex(74) <= ALUSrc;
                id_ex(73) <= RegDst;
                id_ex(72 downto 57) <= if_id(31 downto 16);
                id_ex(56 downto 41) <= RD1;
                id_ex(40 downto 25) <= RD2;
                id_ex(24 downto 9) <= Ext_Imm;
                id_ex(8 downto 6) <= if_id(2 downto 0);
                id_ex(5 downto 3) <= if_id(9 downto 7);
                id_ex(2 downto 0) <= if_id(6 downto 4);
            end if;
        end if;
    end process;
    
    --EX/MEM 
    process(en, clk, id_ex)
    begin
        if rising_edge(clk) then
            if en = '1' then
                ex_mem(55 downto 52) <= id_ex(81 downto 78);
                ex_mem(51 downto 36) <= id_ex(72 downto 57) + id_ex(24 downto 9);
                ex_mem(35) <= zero;
                ex_mem(34 downto 19) <= ALURes;
                ex_mem(18 downto 3) <= id_ex(40 downto 25);
                ex_mem(2 downto 0) <= reg_mux;
            end if;
        end if;
    end process;
    
    -- REGDST MUX
    process(id_ex) 
    begin
        if id_ex(73) = '0' then
            reg_mux <= id_ex(5 downto 3);
        else
            reg_mux <= id_ex(2 downto 0);
        end if;
    end process;
    
    --MEM/WB
    process(ex_mem, clk, en, MemData)
    begin
        if rising_edge(clk) then
            if en = '1' then
                mem_wb(36 downto 35) <= ex_mem(55 downto 54);
                mem_wb(34 downto 19) <= MemData;
                mem_wb(18 downto 3) <= ex_mem(34 downto 19);
                mem_wb(2 downto 0) <= ex_mem(2 downto 0);
            end if;
        end if;
    end process;
    
    -- MEMTOREG MUX
    process(mem_wb)
    begin
        if mem_wb(36) = '0' then
            mem_mux <= mem_wb(34 downto 19);
        else
            mem_mux <= mem_wb(18 downto 3);
        end if;
    end process;
    
    -- buttons: reset, enable
    
    --monopulse1: MPG port map(en, btn(0), clk);
    --monopulse2: MPG port map(rst, btn(1), clk);
    
    -- main units
    --inst_IF: IFetch port map(clk, rst, en, BranchAddress, JumpAddress, Jump, PCSrc, Instruction, PCinc);
    --inst_ID: IDecode port map(clk, en, if_id(12 downto 0), mem_mux, mem_wb(35), id_ex(73), ExtOp, RD1, RD2, Ext_imm, func, sa);
    --inst_MC: MainControl port map(if_id(15 downto 13), RegDst, ExtOp, ALUSrc, Branch, Jump, ALUOp, MemWrite, MemtoReg, RegWrite);
    --inst_EX: ExecutionUnit port map(id_ex(56 downto 41), id_ex(40 downto 25), id_ex(24 downto 9), id_ex(8 downto 6), sa, id_ex(74), id_ex(77 downto 75), ALURes, Zero); 
    --inst_MEM: MEM port map(clk, en, ex_mem(34 downto 19), ex_mem(18 downto 3), ex_mem(53), MemData, ALURes1);

    -- pipe - modified signals
    PCSrc <= ex_mem(52) and ex_mem(35);
    BranchAddress <= ex_mem(51 downto 36);
    JumpAddress <= if_id(31 downto 29) & if_id(12 downto 0); 
    
    ------------ !!! JUMP ADDRESS !!! -------------
    
    
    -- WriteBack unit
    --with MemtoReg select
    --    WD <= MemData when '1',
    --          ALURes1 when '0',
    --          (others => '0') when others;

    -- branch control
    --PCSrc <= Zero and Branch;

    -- jump address
    --JumpAddress <= PCinc(15 downto 13) & Instruction(12 downto 0);

   -- SSD display MUX
    with sw(7 downto 5) select
        digits <=  Instruction when "000", 
                   PCinc when "001",
                   RD1 when "010",
                   RD2 when "011",
                   Ext_Imm when "100",
                   ALURes when "101",
                   MemData when "110",
                   WD when "111",
                   (others => '0') when others; 

    --display : SSD port map (clk, digits, an, cat);

    
    -- main controls on the leds
    --led(10 downto 0) <= ALUOp & RegDst & ExtOp & ALUSrc & Branch & Jump & MemWrite & MemtoReg & RegWrite;
    
end Behavioral;