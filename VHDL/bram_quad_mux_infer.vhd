library ieee;
use ieee.std_logic_1164.all;

entity bram_quad_mux is
    generic (
        ADDR_WIDTH : integer := 32; -- Byte-addressed (32 bits)
        DATA_WIDTH : integer := 16; -- HLS BRAM width (16 bits)
        WE_WIDTH   : integer := 2   -- HLS WEN_A; only LSB used for blk_mem wea
    );
    port (
        -- Mode Select: '1' = Training Mode (hbos_top), '0' = Detection Mode (detection_engine)
        sel_train : in std_logic;

        -- ====================================================
        -- PHYSICAL BRAM INTERFACES (PORT A)
        -- ====================================================
        bram_0_ADDR : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_0_DIN  : out std_logic_vector(DATA_WIDTH-1 downto 0);
        bram_0_WE   : out std_logic;
        bram_0_EN   : out std_logic;
        bram_0_DOUT : in  std_logic_vector(DATA_WIDTH-1 downto 0);

        bram_1_ADDR : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_1_DIN  : out std_logic_vector(DATA_WIDTH-1 downto 0);
        bram_1_WE   : out std_logic;
        bram_1_EN   : out std_logic;
        bram_1_DOUT : in  std_logic_vector(DATA_WIDTH-1 downto 0);

        bram_2_ADDR : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_2_DIN  : out std_logic_vector(DATA_WIDTH-1 downto 0);
        bram_2_WE   : out std_logic;
        bram_2_EN   : out std_logic;
        bram_2_DOUT : in  std_logic_vector(DATA_WIDTH-1 downto 0);

        bram_3_ADDR : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_3_DIN  : out std_logic_vector(DATA_WIDTH-1 downto 0);
        bram_3_WE   : out std_logic;
        bram_3_EN   : out std_logic;
        bram_3_DOUT : in  std_logic_vector(DATA_WIDTH-1 downto 0);

        -- ====================================================
        -- TRAINING ENGINE INTERFACES (PORT A)
        -- ====================================================
        train_0_ADDR : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        train_0_DIN  : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        train_0_WE   : in  std_logic_vector(WE_WIDTH-1 downto 0);
        train_0_EN   : in  std_logic;
        train_0_DOUT : out std_logic_vector(DATA_WIDTH-1 downto 0);

        train_1_ADDR : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        train_1_DIN  : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        train_1_WE   : in  std_logic_vector(WE_WIDTH-1 downto 0);
        train_1_EN   : in  std_logic;
        train_1_DOUT : out std_logic_vector(DATA_WIDTH-1 downto 0);

        train_2_ADDR : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        train_2_DIN  : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        train_2_WE   : in  std_logic_vector(WE_WIDTH-1 downto 0);
        train_2_EN   : in  std_logic;
        train_2_DOUT : out std_logic_vector(DATA_WIDTH-1 downto 0);

        train_3_ADDR : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        train_3_DIN  : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        train_3_WE   : in  std_logic_vector(WE_WIDTH-1 downto 0);
        train_3_EN   : in  std_logic;
        train_3_DOUT : out std_logic_vector(DATA_WIDTH-1 downto 0);

        -- ====================================================
        -- DETECTION ENGINE INTERFACES (PORT A - READ-ONLY)
        -- ====================================================
        det_0_ADDR : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        det_0_EN   : in  std_logic;
        det_0_DOUT : out std_logic_vector(DATA_WIDTH-1 downto 0);

        det_1_ADDR : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        det_1_EN   : in  std_logic;
        det_1_DOUT : out std_logic_vector(DATA_WIDTH-1 downto 0);

        det_2_ADDR : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        det_2_EN   : in  std_logic;
        det_2_DOUT : out std_logic_vector(DATA_WIDTH-1 downto 0);

        det_3_ADDR : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        det_3_EN   : in  std_logic;
        det_3_DOUT : out std_logic_vector(DATA_WIDTH-1 downto 0)
    );
end entity bram_quad_mux;

architecture structural of bram_quad_mux is
    type addr_array_t is array (0 to 3) of std_logic_vector(ADDR_WIDTH-1 downto 0);
    type data_array_t is array (0 to 3) of std_logic_vector(DATA_WIDTH-1 downto 0);
    type we_array_t   is array (0 to 3) of std_logic_vector(WE_WIDTH-1 downto 0);
    type ctrl_array_t is array (0 to 3) of std_logic;

    signal b_addr, t_addr, d_addr : addr_array_t;
    signal b_din,  t_din          : data_array_t;
    signal b_dout, t_dout, d_dout : data_array_t;
    signal b_we,   t_we           : we_array_t;
    signal b_en,   t_en,   d_en   : ctrl_array_t;

begin
    -- 1. Gather all inputs into arrays
    t_addr(0) <= train_0_ADDR;  t_addr(1) <= train_1_ADDR;  t_addr(2) <= train_2_ADDR;  t_addr(3) <= train_3_ADDR;
    t_din(0)  <= train_0_DIN;   t_din(1)  <= train_1_DIN;   t_din(2)  <= train_2_DIN;   t_din(3)  <= train_3_DIN;
    t_we(0)   <= train_0_WE;    t_we(1)   <= train_1_WE;    t_we(2)   <= train_2_WE;    t_we(3)   <= train_3_WE;
    t_en(0)   <= train_0_EN;    t_en(1)   <= train_1_EN;    t_en(2)   <= train_2_EN;    t_en(3)   <= train_3_EN;

    d_addr(0) <= det_0_ADDR;    d_addr(1) <= det_1_ADDR;    d_addr(2) <= det_2_ADDR;    d_addr(3) <= det_3_ADDR;
    d_en(0)   <= det_0_EN;      d_en(1)   <= det_1_EN;      d_en(2)   <= det_2_EN;      d_en(3)   <= det_3_EN;

    b_dout(0) <= bram_0_DOUT;   b_dout(1) <= bram_1_DOUT;   b_dout(2) <= bram_2_DOUT;   b_dout(3) <= bram_3_DOUT;

    -- 2. Parameterized Multiplexer Loop (Zero Latency)
    --    HLS emits BYTE addresses (16-bit data → element N at byte 2N).
    --    Standalone blk_mem_gen Port A expects WORD addresses, so we right-shift
    --    the muxed address by 1 (same translation that bram_addr_shift does for
    --    Port B). Without this, BRAM Port A only ever sees even byte addresses,
    --    half the histogram bins collide pairwise, and reads return wrong data.
    GEN_MUXES: for i in 0 to 3 generate
        b_addr(i) <= ('0' & t_addr(i)(ADDR_WIDTH-1 downto 1)) when sel_train = '1'
                else ('0' & d_addr(i)(ADDR_WIDTH-1 downto 1));
        b_din(i)  <= t_din(i)  when sel_train = '1' else (others => '0');
        b_we(i)   <= t_we(i)   when sel_train = '1' else (others => '0');
        b_en(i)   <= t_en(i)   when sel_train = '1' else d_en(i);

        t_dout(i) <= b_dout(i);
        d_dout(i) <= b_dout(i);
    end generate GEN_MUXES;

    -- 3. Scatter outputs from arrays
    bram_0_ADDR <= b_addr(0);  bram_1_ADDR <= b_addr(1);  bram_2_ADDR <= b_addr(2);  bram_3_ADDR <= b_addr(3);
    bram_0_DIN  <= b_din(0);   bram_1_DIN  <= b_din(1);   bram_2_DIN  <= b_din(2);   bram_3_DIN  <= b_din(3);
    bram_0_WE   <= b_we(0)(0);    bram_1_WE   <= b_we(1)(0);
    bram_2_WE   <= b_we(2)(0);    bram_3_WE   <= b_we(3)(0);
    bram_0_EN   <= b_en(0);    bram_1_EN   <= b_en(1);    bram_2_EN   <= b_en(2);    bram_3_EN   <= b_en(3);

    train_0_DOUT <= t_dout(0); train_1_DOUT <= t_dout(1); train_2_DOUT <= t_dout(2); train_3_DOUT <= t_dout(3);
    det_0_DOUT   <= d_dout(0); det_1_DOUT   <= d_dout(1); det_2_DOUT   <= d_dout(2); det_3_DOUT   <= d_dout(3);

end architecture structural;
