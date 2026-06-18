library ieee;
use ieee.std_logic_1164.all;

-- Translates HLS byte-addresses to standalone BRAM word-addresses
-- for the direct PORTB connection (hbos_top → BRAM).
-- HLS ap_memory outputs byte addresses (element N → 2*N for 16-bit data).
-- Standalone BRAM uses word addresses (element N → N).
-- This module right-shifts the address by 1 bit.

entity bram_addr_shift is
    generic (
        ADDR_WIDTH : integer := 32;
        DATA_WIDTH : integer := 16;
        WE_WIDTH   : integer := 2
    );
    port (
        -- From HLS IP (hbos_top hist_X_PORTB)
        hls_addr : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
        hls_din  : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        hls_we   : in  std_logic_vector(WE_WIDTH-1 downto 0);
        hls_en   : in  std_logic;
        hls_clk  : in  std_logic;
        hls_rst  : in  std_logic;
        hls_dout : out std_logic_vector(DATA_WIDTH-1 downto 0);

        -- To BRAM PORTB
        bram_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_din  : out std_logic_vector(DATA_WIDTH-1 downto 0);
        bram_we   : out std_logic_vector(WE_WIDTH-1 downto 0);
        bram_en   : out std_logic;
        bram_clk  : out std_logic;
        bram_rst  : out std_logic;
        bram_dout : in  std_logic_vector(DATA_WIDTH-1 downto 0)
    );
end entity bram_addr_shift;

architecture rtl of bram_addr_shift is
begin
    -- Right-shift address by 1 (byte-address → word-address)
    bram_addr <= '0' & hls_addr(ADDR_WIDTH-1 downto 1);

    -- Everything else passes through unchanged
    bram_din  <= hls_din;
    bram_we   <= hls_we;
    bram_en   <= hls_en;
    bram_clk  <= hls_clk;
    bram_rst  <= hls_rst;
    hls_dout  <= bram_dout;
end architecture rtl;
