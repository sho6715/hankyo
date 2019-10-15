// *************************************************************************
//   ロボット名		： half
//   概要		： initファイル（レジスタ設定ファイル他）
//   注意		： なし
//   メモ		： WORD = BYTE＊2 LONG ＝BYTE＊４
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <init.h>							//レジスタ


//**************************************************
// 定義（define）
//**************************************************




//**************************************************
// 列挙体（enum）
//**************************************************


//**************************************************
// 構造体（struct）
//**************************************************


//**************************************************
// グローバル変数
//**************************************************




//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

// *************************************************************************
//   機能		： クロックのレジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規
// *************************************************************************/
PRIVATE void init_clock(void)
{


	SYSTEM.PRCR.WORD = 0xa50b;		//クロックソース選択の保護の解除

	SYSTEM.PLLCR.WORD = 0x0F00;		/* PLL 逓倍×16 入力1分周 (12.000MHz * 16 = 192MHz)*/
	SYSTEM.PLLCR2.BYTE = 0x00;		/* PLL ENABLE */
	
	SYSTEM.PLLWTCR.BYTE     = 0x0F;		/* 4194304cycle(Default) */

	
	// ICK   : 192/2 = 96MHz 		// システムクロック CPU DMAC DTC ROM RAM
	// PCLKA : 192/2 = 96MHz	 	// 周辺モジュールクロックA ETHERC、EDMAC、DEU
	// PCLKB : 192/4 = 48MHz 		// 周辺モジュールクロックB 上記以外 PCLKB=PCLK
/*	
	SYSTEM.SCKCR.BIT.FCK=0x02;		//FCLK MAX 50MHz  192/4
	SYSTEM.SCKCR.BIT.ICK=0x01;		//ICLK MAX 100MHz 192/2
	SYSTEM.SCKCR.BIT.PSTOP1=0x01;		//BCLK 出力停止
	SYSTEM.SCKCR.BIT.PSTOP0=0x01;		//SDCLK 出力停止
	SYSTEM.SCKCR.BIT.BCK=0x02;		//BCLK MAX 100MHz ICLK以下にする必要がある192/4
	SYSTEM.SCKCR.BIT.PCKA=0x01;		//PCLKA MAX 100MHz 192/2
	SYSTEM.SCKCR.BIT.PCKB=0x02;		//PCLKB MAX 50MHz 192/4
	//上記の設定では正しくclock設定ができないため下記のように一括で設定すること
*/
	SYSTEM.SCKCR.LONG = 0x21C21211;		//FCK1/4 ICK1/2 BCLK停止 SDCLK停止 BCK1/4 PCLKA1/2 PCLKB1/4
/*
	SYSTEM.SCKCR2.BIT.UCK=0x03;		//UCLK MAX 48MHz 192/4
	SYSTEM.SCKCR2.BIT.IEBCK=0x02;		//IECLK MAX 50MHz 192/4
*/
	SYSTEM.SCKCR2.WORD = 0x0032;		/* UCLK1/4 IEBCK1/4 */
	SYSTEM.BCKCR.BYTE = 0x01;		/* BCLK = 1/2 */
	
	SYSTEM.SCKCR3.WORD = 0x0400;		//PLL回路選択

}

// *************************************************************************
//   機能		： IOのレジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規
// *************************************************************************/
PRIVATE void init_io(void)
{
	/* ================== */
	/*  GPIO(汎用入出力)  */
	/* ================== */
	// 出力値 
	PORTA.PODR.BIT.B1			= 0;		//ポートA-1の初期出力0[V]（デバッグ用LED）
	PORTA.PODR.BIT.B3			= 0;		//ポートA-3の初期出力0[V]（デバッグ用LED）
	PORTA.PODR.BIT.B4			= 0;		//ポートA-4の初期出力0[V]（デバッグ用LED）
	PORTA.PODR.BIT.B6			= 0;		//ポートA-6の初期出力0[V]（デバッグ用LED）
	
	PORT3.PODR.BIT.B7			= 0;		//ポート3-7の初期出力0[V]（電源用赤LED）
//	PORT1.PODR.BIT.B4			= 0;		//ポート1-4の初期出力0[V]（電源用緑LED）
	//センサ
	PORTB.PODR.BIT.B0			= 0;		//ポートB-0の初期出力0[V]（センサLED）
	PORTB.PODR.BIT.B1			= 0;		//ポートB-1の初期出力0[V]（センサLED）
	PORTB.PODR.BIT.B3			= 0;		//ポートB-3の初期出力0[V]（センサLED）
	PORTB.PODR.BIT.B5			= 0;		//ポートB-5の初期出力0[V]（センサLED）
	//DCM
	PORT1.PODR.BIT.B7			= 0;		//PWM(TPU0)
	PORT1.PODR.BIT.B6			= 0;		//PWM(TPU3)
	PORT3.PODR.BIT.B1			= 0;		//AIN1
	PORT1.PODR.BIT.B5			= 0;		//AIN2
//	PORTB.PODR.BIT.B5			= 0;		//BIN1
//	PORTB.PODR.BIT.B6			= 0;		//BIN2
//	PORTB.PODR.BIT.B7			= 0;		//STBY
	
	// 入出力設定 
	PORTA.PDR.BIT.B1			= 1;		//ポートA-1を出力に設定
	PORTA.PDR.BIT.B3			= 1;		//ポートA-3を出力に設定
	PORTA.PDR.BIT.B4			= 1;		//ポートA-4を出力に設定
	PORTA.PDR.BIT.B6			= 1;		//ポートA-6を出力に設定
	
	PORT3.PDR.BIT.B7			= 1;		//ポート3-7を出力に設定
//	PORT1.PDR.BIT.B4			= 1;		//ポート1-4を出力に設定
	//センサ
	PORTB.PDR.BIT.B0			= 1;		//ポートB-0を出力に設定
	PORTB.PDR.BIT.B1			= 1;		//ポートB-1を出力に設定
	PORTB.PDR.BIT.B3			= 1;		//ポートB-3を出力に設定
	PORTB.PDR.BIT.B5			= 1;		//ポートB-5を出力に設定
	//DCM
	PORT1.PDR.BIT.B7			= 1;		//PWM(TPU0)
	PORT1.PDR.BIT.B6			= 1;		//PWM(TPU3)
	PORT3.PDR.BIT.B1			= 1;		//AIN1
	PORT1.PDR.BIT.B5			= 1;		//AIN2
//	PORTB.PDR.BIT.B5			= 1;		//BIN1
//	PORTB.PDR.BIT.B6			= 1;		//BIN2
//	PORTB.PDR.BIT.B7			= 1;		//STBY
	
	PORT3.PDR.BIT.B1			= 1;		//AIN2
	// 入力プルアップ設定 
	PORT2.PCR.BIT.B7			= 1;		// ポート2-7はプルアップを使用	(プッシュスイッチ用)
	PORT4.PCR.BIT.B6			= 1;		// ポート4-6はプルアップを使用	(プッシュスイッチ用)
	
	// ポート入力データレジスタ 
	PORT2.PIDR.BIT.B7			= 1;		// ポート2-7を入力ポートと接続する
	PORT4.PIDR.BIT.B6			= 1;		// ポート4-6を入力ポートと接続する
	
	
	/* ========= */
	/*  A/D変換  */
	/* ========= */
	SYSTEM.PRCR.WORD = 0xA502;
	SYSTEM.MSTPCRA.BIT.MSTPA17 	= 0;		// AD(12bit)スタンバイ解除
	SYSTEM.PRCR.WORD = 0xA500;
	
	S12AD.ADCSR.BIT.CKS 		= 3;		// PCLKで変換
	
	
	//TPUPWM用
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P17PFS.BIT.PSEL		= 3;	//TIOCB0
	MPC.P16PFS.BIT.PSEL		= 3;	//TIOCB1
	MPC.PWPR.BIT.B0WI  = 1;
	MPC.PWPR.BIT.PFSWE = 0;
	PORT1.PMR.BIT.B7		= 1;
	PORT1.PMR.BIT.B6		= 1;
	
	
}

// *************************************************************************
//   機能		： MTUのレジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規
// *************************************************************************/
PRIVATE void init_mtu(void)
{
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(MTU0) 	= 0;						// 
	MSTP(MTU1) 	= 0;
	MSTP(MTU3) 	= 0;
	MSTP(MTU2)	= 0;
	SYSTEM.PRCR.WORD = 0xA500;
	
	MTU.TSTR.BYTE = 0;						//タイマ動作ストップ
	
	// -----------------------
	//  システム用(MTU0)
	// -----------------------
	// タイマ割り込みの設定 
	MTU0.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	MTU0.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 で1カウント
	MTU0.TIER.BIT.TGIEA			= 1;		// TGRAとのコンペアマッチで割り込み許可
	MTU0.TGRA 					= 750 * 4;	// 1msec毎に割り込み
	MTU0.TCNT 					= 0;		// タイマクリア
	
	IEN(MTU0,TGIA0) = 1;	//割り込み要求を許可 
	IPR(MTU0,TGIA0) = 7;	//割り込み優先度を次点に設定
	IR(MTU0,TGIA0)	= 0;	//割り込みステータスフラグをクリア
	
	MTU.TSTR.BIT.CST0 = 0;	//タイマストップ
	
	// -----------------------
	//  バッテリー用(MTU1)
	// -----------------------
	// タイマ割り込みの設定 
	MTU1.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	MTU1.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 で1カウント
	MTU1.TIER.BIT.TGIEA			= 1;		// TGRAとのコンペアマッチで割り込み許可
	MTU1.TGRA 					= 7500 * 4;	// 10msec毎に割り込み
	MTU1.TCNT 					= 0;		// タイマクリア
	
	IEN(MTU1,TGIA1) = 1;	//割り込み要求を許可 
	IPR(MTU1,TGIA1) = 6;	//割り込み優先度を次点に設定
	IR(MTU1,TGIA1)	= 0;	//割り込みステータスフラグをクリア
	
	MTU.TSTR.BIT.CST1 = 0;	//タイマストップ
	
	// -----------------------
	//  センサ用(MTU3)
	// -----------------------
	// タイマ割り込みの設定 
	MTU3.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	MTU3.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 で1カウント
	MTU3.TIER.BIT.TGIEA			= 1;		// TGRAとのコンペアマッチで割り込み許可
	MTU3.TGRA 					= 750;	// 250usec毎に割り込み
	MTU3.TCNT 					= 0;		// タイマクリア
	
	IEN(MTU3,TGIA3) = 1;	//割り込み要求を許可 
	IPR(MTU3,TGIA3) = 8;	//割り込み優先度を次点に設定
	IR(MTU3,TGIA3)	= 0;	//割り込みステータスフラグをクリア
	
	MTU.TSTR.BIT.CST3 = 0;	//タイマストップ
	
}

// *************************************************************************
//   機能		： TPUのレジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.07.03			sato			新規
// *************************************************************************/
PRIVATE void init_tpu(void)
{
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(TPU0) 			= 0;
	MSTP(TPU1) 			= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	
	// -----------------------
	//  右モータPWM出力用(TPU0)
	// -----------------------
	TPU0.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	TPU0.TCR.BIT.CKEG			= 1;		// CKEG立ち上がりエッジでカウント
	TPU0.TCR.BIT.TPSC 			= 1;		// PCLK(48MHz)/4 で1カウント
	TPU0.TMDR.BIT.MD			= 3;		// PWM モード 2
//	TPU0.TIORH.BIT.IOA			= 2;		// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
	TPU0.TIORH.BIT.IOB			= 2;		// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
	TPU0.TGRA 				= 240;		// 周期(usec)
	TPU0.TGRB 				= 120;		// onDuty
	TPU0.TCNT 				= 0;		// タイマクリア
	
	// -----------------------
	//  左モータPWM出力用(TPU1)
	// -----------------------
	TPU1.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	TPU1.TCR.BIT.CKEG			= 1;		// CKEG立ち上がりエッジでカウント
	TPU1.TCR.BIT.TPSC 			= 1;		// PCLK(48MHz)/4 で1カウント
	TPU1.TMDR.BIT.MD			= 3;		// PWM モード 2
//	TPU1.TIORH.BIT.IOA			= 2;		// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
	TPU1.TIOR.BIT.IOB			= 2;		// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
	TPU1.TGRA 				= 240;		// 周期(usec)
	TPU1.TGRB 				= 120;		// onDuty
	TPU1.TCNT 				= 0;		// タイマクリア
	
}

// *************************************************************************
//   機能		： SCIのレジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.07.03			sato			新規
// *************************************************************************/
PRIVATE void init_sci(void)
{	
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(SCI1) 			= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	SCI1.SCR.BYTE	= 0x00;
	while(0x00 != (SCI1.SCR.BYTE & 0xF0));
	SCI1.SMR.BYTE	= 0x00;
//	PORT2.PODR.BIT.B6 = 1;			//TXDのDirctionの切り替え後の値をhigh
//	PORT2.PDR.BIT.B6 = 1;			//出力に設定
//	PORT3.PDR.BIT.B0 = 0;			//入力に設定
//	PORT2.PMR.BIT.B6 = 0;			//汎用ポートに設定
//	PORT3.PMR.BIT.B0 = 0;			//汎用ポートに設定
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P26PFS.BIT.PSEL = 0x0A;		//TXD1
	MPC.P30PFS.BIT.PSEL = 0x0A;		//RXD1
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI  = 1;
	PORT3.PMR.BIT.B0 = 1;			//周辺機能(RXD1)として使用
	PORT2.PMR.BIT.B6 = 1;			//周辺機能(TXD1)として使用
	SCI1.BRR	= 78;
	SCI1.SEMR.BIT.ABCS = 1;
	SCI1.SCR.BYTE	=0x30;

	IEN(SCI1,RXI1) 		= 1;
	IEN(SCI1,TXI1) 		= 1;
	ICU.IPR[217].BIT.IPR	= 5;

}

// *************************************************************************
//   機能		： SPI設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規
// *************************************************************************/
PUBLIC void init_spi(void)
{
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(RSPI0) 		= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	
	RSPI0.SPCR.BYTE = 0x00;
	
//	PORTC.PMR.BIT.B4	= 1;
	PORTC.PMR.BIT.B5	= 1;
	PORTC.PMR.BIT.B6	= 1;
	PORTC.PMR.BIT.B7	= 1;
	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
//	MPC.PC4PFS.BYTE = 0x0D;
	MPC.PC5PFS.BYTE = 0x0D;  /* RSPI RSPCKA */       
	MPC.PC6PFS.BYTE = 0x0D;  /* RSPI MOSIA  */
	MPC.PC7PFS.BYTE = 0x0D;  /* RSPI MISOA  */
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI = 1;
	PORTC.PODR.BIT.B4 = 1;
	PORTC.PDR.BIT.B4 = 1;
//エンコーダ用
	PORT4.PODR.BIT.B2 = 1;
	PORT4.PDR.BIT.B2 = 1;
	PORT4.PODR.BIT.B1 = 1;
	PORT4.PDR.BIT.B1 = 1;
	
	RSPI0.SPBR		= 2;
	RSPI0.SPCMD0.WORD	= 0x0f83;	
	
	RSPI0.SPCR.BYTE = 0xF8;

}

// *************************************************************************
//   機能		： レジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規
// *************************************************************************/
PUBLIC void CPU_init(void)
{
	init_clock();
	init_io();
	init_mtu();
	init_tpu();
	init_sci();
	init_spi();
}
