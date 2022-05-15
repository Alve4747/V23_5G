#ifndef __VIVO_SMARTPA_CALI__
#define __VIVO_SMARTPA_CALI__

#define VIVO_TRANSF_IMPED_TO_USER_I(X) (((X * 100) >> 15) / 100)
#define VIVO_TRANSF_IMPED_TO_USER_M(X) (((X * 100) >> 15) % 100)

/* the following use for kernel */
struct smartpa_param {
	uint32_t PA_ID;//vendor ID
	uint32_t Imped_Min_L15;     // read from dts
	uint32_t Imped_Max_L15;     // read from dts
	uint32_t Freq_Min_L15;      // read from dts
	uint32_t Freq_Max_L15;      // read from dts
	uint32_t Qt_Min_L15;        // read from dts
	uint32_t V_Max_L15;         // read from dts
	uint32_t I_Max_L15;         // read from dts
	uint32_t Vout_Max_L15;      // read from dts
	uint32_t Re_L15;        //calculate Result
	uint32_t F0_L15;        //calculate Result
	uint32_t Qts_L15;       //calculate Result
	uint32_t Calibration_Success;
};

/* the following use for communication with ADSP */
struct CALIBRATION_RX_{
	uint32_t Re_L15;
	uint32_t F0_L15;
	uint32_t Qts_L15;
	uint32_t Calibration_Success;
	uint32_t PA_ID;
	uint32_t V_Max_L15;
	uint32_t I_Max_L15;
	uint32_t Vout_Max_L15;
};

struct vivo_cali_ops {
	int (*vivo_smartpa_init_dbg)(char *buffer, int size);
	int (*vivo_smartpa_read_freq_dbg)(char *buffer, int size);
	int (*vivo_smartpa_check_calib_dbg)(void);
	void (*vivo_smartpa_read_prars_dbg)(int temp[5], unsigned char addr);
};

struct mtk_apr {
	uint32_t param_id;
	uint32_t data[14];
	struct CALIBRATION_RX_ calib_param;
};

/* the following use for communication with up layer */
#define MSGS_SIZE 256
#define RESERVED_SIZE 252
struct smartpa_msg {
	char msgs[MSGS_SIZE];
    char reserved[RESERVED_SIZE];
    int msg_result;
};

struct smartpa_prars {
	int fRes_max;
	int fRes_min;
	int Qt;
	int impedance_max;
	int impedance_min;
};


#define MAX_DSP_PARAM_INDEX		881//600
#define MAX_CHANNELS	2

#define CALIB_FAILED 0xCACACACA

#define AP_2_DSP_SEND_PARAM     2
#define AP_2_DSP_GET_PARAM		1
#define AP_2_DSP_SET_PARAM		0
#define AP_2_DSP_PAYLOAD_SIZE	14
#define AP_2_DSP_RX_PORT_ID		0x1004 /* TERT MI2S RX */
#define AP_2_DSP_TX_PORT_ID		0x1005 /* TERT MI2S TX */
#define SLAVE1		0x98
#define SLAVE2		0x9A
#define SLAVE3		0x9C
#define SLAVE4		0x9E
#define SMARTAMP_STATUS_NORMAL 0
#define SMARTAMP_STATUS_BYPASS 1
#define SMARTAMP_STATUS_MUTE   2
#define AFE_SA_SEND_ALL        3808
#define AFE_SA_GET_F0          3810
#define AFE_SA_GET_Q           3811
#define AFE_SA_GET_TV          3812
#define AFE_SA_GET_RE          3813
#define AFE_SA_CALIB_INIT      3814
#define AFE_SA_CALIB_DEINIT    3815
#define AFE_SA_SET_RE          3816
#define AFE_SA_F0_TEST_INIT    3817
#define AFE_SA_F0_TEST_DEINIT  3818
#define AFE_SA_SET_PROFILE     3819
#define AFE_SA_SET_STATUS      3820
#define AFE_SA_BYPASS		   3821

/* the following are interfaces for other modules to use */
int vivo_smartpa_debug_probe(struct vivo_cali_ops *ops);

int mtk_afe_smartamp_algo_ctrl(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint8_t length);

#endif
