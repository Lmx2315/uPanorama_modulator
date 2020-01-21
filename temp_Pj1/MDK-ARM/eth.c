
#include "main.h"

#define RX_MAX_BUF_SIZE	2048
#define TX_MAX_BUF_SIZE 2048

u8 TX_BUF[TX_MAX_BUF_SIZE];
u8 RX_BUF[RX_MAX_BUF_SIZE];

u32   TX_BUF_LENGTH=0;

u32 destip_UDP    	 =0; //IP корреспондента UDP
u16 destport_UDP  	 =0; //port корреспондента UDP
u16 FLAG_UDP_EVENT	 =0u;
u8 FLAG_UDP_INSTALL	 =0u;

extern  Frame INVOICE[quantity_SENDER];	//структура квитанций о состоянии дел, по числу потенциальный адресатов
extern  Frame FRM1   [quantity_SENDER];	//принятый фрейм, по числу потенциальный адресатов
extern  SERVER SERV1;					//структура "Хранилище"
extern  ID_SERVER ID_SERV1;				//структура указатель для "Хранилище"
extern  CMD_RUN CMD1;
extern  ADR_SENDER ADDR_SNDR;			//структура хранит массив адресов ОТПРАВИТЕЛЕЙ 
extern  u64 TIME_SYS;					//переменная хранить системное время в милисекундах
extern  u32 ID_CMD;						//переменная хранить текущий ID наших квитанций 


//----------------------------------------------------------------
void ETH0_MAC_write (u64 temp)
{
	u_out("write my    MAC:",temp);

	 FPGA_wSPI(
	 48,		//размерность посылаемых данных в битах
	 46 ,		//адрес на внутренней шине в ПЛИС ETH0
	 temp		//данные
	 );//записываем в ETH0 наш MAC адрес
}

void ETH0_PORT_write (u32 PORT_my,u16 PORT_dest)
{
	u32 temp;
		temp     =(PORT_my  <<16)+(PORT_dest); 	 
		u_out("write my   Port:",PORT_my);
		u_out("write dest Port:",PORT_dest);

	 FPGA_wSPI(
	 32,		//размерность посылаемых данных в битах
	 45 ,		//адрес на внутренней шине в ПЛИС ETH0
	 temp		//данные
	 );			//записываем в ETH0  номер порта , наш и не наш
}

void ETH0_IP_write (u64 IP_my,u32 IP_dest)
{
 u64 temp     =(IP_my  <<32)+(IP_dest);
	x_out("write my     IP:",IP_my);
	x_out("write dest   IP:",IP_dest);

	 FPGA_wSPI(
	 64,		//размерность посылаемых данных в битах
	 21,		//адрес на внутренней шине в ПЛИС ETH0
	 temp		//данные
	 );			//записываем в ETH0 наш IP и наш номер порты
}

void ETH1_MAC_write (u64 temp)
{
	u_out("write my    MAC:",temp);

	 FPGA_wSPI(
	 48,		//размерность посылаемых данных в битах
	 47 ,		//адрес на внутренней шине в ПЛИС ETH0
	 temp		//данные
	 );//записываем в ETH1 наш MAC адрес
}

void ETH1_PORT_write (u32 PORT_my,u16 PORT_dest)
{
	u32 temp;
		temp     =(PORT_my  <<16)+(PORT_dest); 	 
		u_out("write my   Port:",PORT_my);
		u_out("write dest Port:",PORT_dest);

	 FPGA_wSPI(
	 32,		//размерность посылаемых данных в битах
	 23 ,		//адрес на внутренней шине в ПЛИС ETH0
	 temp		//данные
	 );			//записываем в ETH1  номер порта , наш и не наш
}

void ETH1_IP_write (u64 IP_my,u32 IP_dest)
{
 u64 temp     =(IP_my  <<32)+(IP_dest);
	x_out("write my     IP:",IP_my);
	x_out("write dest   IP:",IP_dest);

	 FPGA_wSPI(
	 64,		//размерность посылаемых данных в битах
	 22,		//адрес на внутренней шине в ПЛИС ETH0
	 temp		//данные
	 );			//записываем в ETH1 наш IP и наш номер порты
}

//--------------------------------------------------------------

void SEND_udp (u16 dest_Port,u32 dest_IP,u8 *m,u16 size)
{
u16 i=0;
u32 data=0;
u16 adr=0;

u8 d0=0;
u8 d1=0;
u8 d2=0;
u8 d3=0;

u8 k=0;
u32 crc=0;

Transf("\r\n---------\r\n");
Transf("Заполняем МЕМ3>\r\n");


for (i=0;i<size;i++)
	{
			
		if (k==0) d0=m[i];
		if (k==1) d1=m[i];
		if (k==2) d2=m[i];
		if (k==3) d3=m[i];
		
		if (k==3) 
		{			
			data=    (d0<<24)+(d1<<16) + (d2<< 8)+(d3<<0);
			crc=crc+((d0<< 8)+(d1<< 0))+((d2<< 8)+(d3<<0));//число байт данных должно быть чётно!!! чтобы можно было сложить в 16 бит
			FPGA_wSPI (16,38, adr);//записываем адресс памяти
			FPGA_wSPI (32,39,data);//записываем данные памяти	
			
			un_out("",adr);
			Transf(":\r\n");
		    hn_out(data,24);hn_out(data,16);hn_out(data,8);h_out(data,0);
			adr++;
		}		
		
		if (k<3) k++; else k=0;	
		//Transf(".");
	}
	
	FPGA_wSPI (32,44, crc);//записываем CRC данных в памяти
	
Transf("\r\n");	
Transf("\r\n---------\r\n");	
}

u32 ADR_FINDER (u64 adr,ADR_SENDER *a) //ищем адрес отправителя в структуре адресов (его порядковый номер)
{
	u32 i=0;
	u32 n=0;

	n=a->IDX;
	//ищем среди уже записаных адресов
	while (i<quantity_SENDER)
	{
		if (adr==a->A[i]) break;
		i++;
	}

	if ((i==quantity_SENDER)&&(n==quantity_SENDER)) //проверяем на переполнение списка отправителей
	{
		Transf("Превышено число ОТПРАВИТЕЛЕЙ!\r\n");//записываем отправителя в нулевой адрес структуры
		i=0;
		a->A[0]=adr;
	} else 
		if (i==quantity_SENDER)  //если не нашёл такой адрес в памяти
	{		
		Transf("Новый адрес!\r\n");
		a->A[a->IDX]=adr;	
		   i=a->IDX;
		   	 a->IDX++;
	} 
		
//	u_out("n:",n);	
//	u_out("a->IDX:",a->IDX);
//	u_out("   adr:",adr);
	
	return i;
}

//-----------------------------------------------------------------
//             тестовый вывод "Хранилища"
void PRINT_SERV (void)
{
	u32 i=0;
	u32 j=0;
	Transf("\r\n");
	
	for (i=0;i<SIZE_SERVER;i++)
	{
		if (j<4) 
		{
			if (SERV1.MeM[i]<10) xn_out("  ",SERV1.MeM[i]); 
			else				 xn_out(" " ,SERV1.MeM[i]);
		    if (j==3) {Transf("\r\n");j=0;}	else j++;		
		}
	}	
	Transf("\r\n");	
}


void PRINT_SERV_ID (void)
{
	u32 i=0;
	
	Transf("\r\n");
	x_out("N_sch    :",ID_SERV1.N_sch);
	
	for (i=0;i<SIZE_ID;i++)
	{
		if (ID_SERV1.INDEX[i]!=0xffffffff) 
		{		
			Transf("\r\n");	
			u_out("INDEX    :",ID_SERV1.INDEX      [i]);
			x_out("CMD_TYPE :0x",ID_SERV1.CMD_TYPE [i]);
			x_out("SENDER_ID:0x",ID_SERV1.SENDER_ID[i]);
			u_out("TIME     :",ID_SERV1.TIME       [i]);			
		}
	}		
}

void FRAME_DECODE (u8 *M,u32 len)
{
	u64 i=0;
	u32 j=0;
	u32 k=0;
	u32 offset=0;
	u32 INDEX_archiv=0;
	u32 ADR=0;//индекс отправителя в структуре адресов отправителей
	u64 SENDER=0;
	
//----------------------это шапка  Фрейма---------------------------------------------	
	SENDER						=((u64)M[ 8]<<56)|((u64)M[ 9]<<48)|((u64)M[10]<<40)|((u64)M[11]<<32)|(M[12]<<24)|(M[13]<<16)|(M[14]<< 8)|(M[15]<<0);//получаем адрес отправителя
	
	ADR=ADR_FINDER (SENDER,&ADDR_SNDR);//ищем порядковый номер отправителя в структуре отправителей, если его там нет  - то заносим туда
//	u_out("ADR:",ADR);
  	FRM1[ADR].Frame_size		=     (M[ 0]<< 8)|     (M[ 1]<<0);
	FRM1[ADR].Frame_number		=     (M[ 2]<< 8)|     (M[ 3]<<0);//где-то среди этих байтов есть STOP_BIT
	FRM1[ADR].Msg_uniq_id		=     (M[ 4]<<24)|     (M[ 5]<<16)|     (M[ 6]<< 8)|     (M[ 7]<<0);
	FRM1[ADR].Sender_id			=((u64)M[ 8]<<56)|((u64)M[ 9]<<48)|((u64)M[10]<<40)|((u64)M[11]<<32)|(M[12]<<24)|(M[13]<<16)|(M[14]<< 8)|(M[15]<<0);
	FRM1[ADR].Receiver_id		=((u64)M[16]<<56)|((u64)M[17]<<48)|((u64)M[18]<<40)|((u64)M[19]<<32)|(M[20]<<24)|(M[21]<<16)|(M[22]<< 8)|(M[23]<<0);
	
	FRM1[ADR].MSG.Msg_size		=     (M[24]<<24)|     (M[25]<<16)|     (M[26]<< 8)|     (M[27]<<0);
	FRM1[ADR].MSG.Msg_type		=     (M[28]<<24)|     (M[29]<<16)|     (M[30]<< 8)|     (M[31]<<0);
	FRM1[ADR].MSG.Num_cmd_in_msg=((u64)M[32]<<56)|((u64)M[33]<<48)|((u64)M[34]<<40)|((u64)M[35]<<32)|(M[36]<<24)|(M[37]<<16)|(M[38]<< 8)|(M[39]<<0);  
//------------------------------------------------------------------------------------	 
	
	offset=40;
	for (i=0;i<FRM1[ADR].MSG.Num_cmd_in_msg;i++)
	{			
		FRM1[ADR].MSG.CMD[i].Cmd_size	=     (M[offset+ 0]<<24)|     (M[offset+ 1]<<16)|     (M[offset+ 2]<< 8)|     (M[offset+ 3]<<0);		
		FRM1[ADR].MSG.CMD[i].Cmd_type	=	  (M[offset+ 4]<<24)|     (M[offset+ 5]<<16)|     (M[offset+ 6]<< 8)|     (M[offset+ 7]<<0);	
		FRM1[ADR].MSG.CMD[i].Cmd_id		=((u64)M[offset+ 8]<<56)|((u64)M[offset+ 9]<<48)|((u64)M[offset+10]<<40)|((u64)M[offset+11]<<32)|(M[offset+12]<<24)|(M[offset+13]<<16)|(M[offset+14]<< 8)|(M[offset+15]<<0);
		FRM1[ADR].MSG.CMD[i].Cmd_time	=((u64)M[offset+16]<<56)|((u64)M[offset+17]<<48)|((u64)M[offset+18]<<40)|((u64)M[offset+19]<<32)|(M[offset+20]<<24)|(M[offset+21]<<16)|(M[offset+22]<< 8)|(M[offset+23]<<0);
		
		INDEX_archiv	=SERV1.INDEX;							//запоминаем текущий индекс "хранилища" в прошлый
		SERV1.INDEX_LAST=SERV1.INDEX;
		
		SERV1.x1=SERV1.INDEX-1;									//запоминаем начало интервала удалённых индексов для реестра
		
	 for (k=0;k<24;k++)	SERV_WR (M[offset+ k],&SERV1);			//переписываем пришедшие пакеты в "Хранилище"
		
	 for (j=0;j<FRM1[ADR].MSG.CMD[i].Cmd_size;j++)
		{
			FRM1[ADR].MSG.CMD[i].Cmd_data[j]=M[offset+24+j];
			SERV_WR (M[offset+24+j],&SERV1);					//переписываем пришедшие пакеты в "Хранилище"
		} 
		
		SERV1.x2=SERV1.INDEX+1;									//запоминаем конец интервала удалённых индексов для реестра
		
		offset=offset+24+j;
/**/		SERV_ID_WR (  &INVOICE[ADR],					//указатель на структуру квитанции
							     &SERV1,					//указатель на Хранилище   
						      &ID_SERV1,					//указатель на реестр
						   INDEX_archiv, 					//заполняем массив ID
					    FRM1[ADR].MSG.CMD[i].Cmd_type,
						FRM1[ADR].Sender_id,
						FRM1[ADR].MSG.CMD[i].Cmd_time,
						FRM1[ADR].MSG.CMD[i].Cmd_id
						);

	}	
	
//-------------------отладка--------------------------	

 	Transf("\r\n");
	u_out("Frame_size    :",FRM1[ADR].Frame_size);
	u_out("Frame_number  :",FRM1[ADR].Frame_number);
	u_out("Msg_uniq_id   :",FRM1[ADR].Msg_uniq_id);
	u_out("Sender_id     :",FRM1[ADR].Sender_id);
	u_out("Receiver_id   :",FRM1[ADR].Receiver_id);
	u_out("Msg_size      :",FRM1[ADR].MSG.Msg_size);
	u_out("Msg_type      :",FRM1[ADR].MSG.Msg_type);
	u_out("Num_cmd_in_msg:",FRM1[ADR].MSG.Num_cmd_in_msg);
	
	
	  for (i=0;i<FRM1[ADR].MSG.Num_cmd_in_msg;i++)
	{
		Transf("\r\n");
		u_out("Cmd_size:",FRM1[ADR].MSG.CMD[i].Cmd_size);
		u_out("Cmd_type:",FRM1[ADR].MSG.CMD[i].Cmd_type);
		u_out("Cmd_id  :",FRM1[ADR].MSG.CMD[i].Cmd_id);
		u_out("Cmd_time:",FRM1[ADR].MSG.CMD[i].Cmd_time);
		Transf("\r\n");

		for (j=0;j<FRM1[ADR].MSG.CMD[i].Cmd_size;j++)
		{
			x_out("Cmd_data:0x",FRM1[ADR].MSG.CMD[i].Cmd_data[j]);
		}	 
	} 	
		PRINT_SERV    ();//выводим "Хранилище"
		PRINT_SERV_ID ();//выводим ID
		STATUS_ID (&ID_SERV1); 
}

void INIT_SERV_ARCHIV (SERVER *s,ID_SERVER *idx,ADR_SENDER *adr)
{
	u32 i=0;
//--------------------------------------------
//     инициализируем счётчик отправителей 	
	for (i=0;i<quantity_SENDER;i++)
	{
		adr->A[i]=0xffffffff;
	}
	adr->IDX=0;
//--------------------------------------------
//      инициализируем реестр	
	for (i=0;i<SIZE_ID;i++)
	{
		idx->INDEX   [i]=0xffffffff;//очищаем массив индексов
		idx->CMD_TYPE[i]=0xffffffffffffffff;
		idx->TIME    [i]=0xffffffffffffffff;
		idx->FLAG_REAL_TIME[i]=0;
	}
	idx->N_sch=0;
//--------------------------------------------
//      инициализируем ХРАНИЛИЩЕ	
	for (i=0;i<SIZE_SERVER;i++)
	{
		s->MeM[i]=0x00;//очищаем массив индексов
	}	
	s->INDEX		=0;
	s->INDEX_LAST	=0;
	s->CMD_ID		=0;
	s->SENDER_ID	=0;
	s->TIME			=0;
}

void SERV_WR (u8 a,SERVER *s)
{
	u32 i=s->INDEX;	
		  s->MeM[i]=a;
	  if (s->INDEX==(SIZE_SERVER-1)) s->INDEX=0; else  s->INDEX= s->INDEX+1;	  
}

void MSG_SHOW (void)
{
	u32 i=0;
	u32 j=0;
	Transf("----------\r\n");
	
	for (i=0;i<quantity_SENDER;i++)
	{
		u_out("SENDER:",i);
		u_out("Число квитанций:",INVOICE[i].MSG.Num_cmd_in_msg);
		for (j=0;j<INVOICE[i].MSG.Num_cmd_in_msg;j++)
		{
		   u_out("номер квитанции:",j);
		   u_out("CMD:",INVOICE[i].MSG.CMD[j].Cmd_type);
		}
	}
	
}

u32 SERV_ID_WR  //функция заполнения реестра команд
(
Frame  *m,		 //указатель на структуру квитанции
SERVER *srv,     //указатель на структуру "Хранилище"
ID_SERVER *s,	 //указатель на структуру "реестр"
u32 a,			 //индекс в Хранилище
u32 b,			 //CMD_TYPE
u32 c,			 //SENDER_ID
u64 d,			 //TIME
u64 e			 //CMD_ID
) //при записи увеличиваем счётчик записи, при очистке - сбрасываем.
{
	u32 i=0;
	u32 j=0;
	u32 error=0;
	u32 ERROR_ID=0;
	//------ищем затёртые пачки в хранилище и выкидываем их из реестра

		for (j=0;j<SIZE_ID;j++)
		{
			if (s->INDEX[j]!=0xffffffff)//проверяем что индекс действительный (0xffffffff - пустое место)
			{
				if (srv->x1<srv->x2)
				{//------это вариант когда пакет нормально в буфере расположен-----
					if((s->INDEX[j]>srv->x1)&&(s->INDEX[j]<srv->x2)) //нашли индекс команды в реестре, которая была затёрта в кольцевом буфере 
					{
					//	Transf("v1\r\n");
						ERROR_ID = ERROR_CMD_BUF;				//команда не выполненна потому что была затёрта в буфере
						Transf("квитанция о затёртых пакетах!\r\n");
						ERROR_CMD_MSG(s,m,j,ERROR_ID,0,0);	//заполняем квитанцию о событии
						s->INDEX	[j] =0xffffffff;	
						s->CMD_TYPE	[j] =0;
						s->SENDER_ID[j]	=0;
						s->TIME		[j]	=0;
						s->N_sch--;
					}
				} else
				{//----это вариант когда пол пакета в конце буфера и пол пакета в начале----
					if((s->INDEX[j]>srv->x1)||(s->INDEX[j]<srv->x2)) //нашли индекс команды в реестре, которая была затёрта в кольцевом буфере 
					{
					//	Transf("v2\r\n");
					//	u_out("s->INDEX[j]:",s->INDEX[j]);
					//	u_out("    srv->x1:",srv->x1);
					//	u_out("    srv->x2:",srv->x2);
						
						ERROR_ID = ERROR_CMD_BUF;				//команда не выполненна потому что была затёрта в буфере
						Transf("квитанция о затёртых пакетах!\r\n");
						u_out("s->CMD_TYPE	[j]:",s->CMD_TYPE	[j]);
						ERROR_CMD_MSG(s,m,j,ERROR_ID,0,0);	//заполняем квитанцию о событии
						s->INDEX	[j] =0xffffffff;	
						s->CMD_TYPE	[j] =0;
						s->SENDER_ID[j]	=0;
						s->TIME		[j]	=0;
						s->N_sch--;
					}
				}				
			}			
		}	
	
	//-----Записываем в реестр новую команду на свободное место
	i=0;
	if (s->N_sch<SIZE_ID)
	{
		while (s->INDEX[i]!=0xffffffff) {i++;};//ищем не занятую строчку в реестре
	
		if (b==CMD_TIME_SETUP) s->FLAG_REAL_TIME[i]=1;//команда реального времени , тут 001 - установка времени, должна производиться с привязкой к секундной метке
		else 	  			   s->FLAG_REAL_TIME[i]=0;
		s->INDEX	[i]	=a;
		s->CMD_TYPE	[i] =b;
		s->CMD_ID   [i] =e;
		s->SENDER_ID[i]	=c;
		s->TIME		[i]	=d;
		s->N_sch++;
	} else error=1;
			
return error; //возвращаем сигнал аварии если буфер переполнен
}

void SERV_ID_DEL (ID_SERVER *id,u32 idx) //процедура удаления команды из реестра
{
	id->INDEX	 [idx] =0xffffffff;
	id->CMD_TYPE [idx] =0xffffffff;
	id->CMD_ID   [idx] =0xffffffff;
	id->SENDER_ID[idx] =0xffffffff;
	id->TIME	 [idx] =0xffffffff;
	id->N_sch--;
	u_out("удаляем команду из реестра:",idx);
}

void STATUS_ID (ID_SERVER *id)
{
	Transf("--------\r\n");
	u32 i=0;
	for (i=0;i<SIZE_ID;i++)
	{
		un_out("    INDEX[",i);Transf("]:");u_out("",id->INDEX    [i]);
		un_out(" CMD_TYPE[",i);Transf("]:");x_out("",id->CMD_TYPE [i]);
		un_out("SENDER_ID[",i);Transf("]:");x_out("",id->SENDER_ID[i]);
		un_out("     TIME[",i);Transf("]:");u_out("",id->TIME     [i]);
		u_out("N:",id->N_sch);
		Transf("\r\n");
	}
}

//эта функция обслуживает заполнение квитанций
u32 ERROR_CMD_MSG(
ID_SERVER *s,//реестр
Frame *invc, //структура квитанций	
u32 INDEX,	 //индекс в реестре
u32 MSG_TYPE,//тип сообщения
u32 DATA_MSG,//данные сообщения
u32 time	 //время составления квитанции
)
{
//	Transf("Заполняем квитанцию!\r\n");
	u32 STATUS=0;//успешно выполнили заполнение квитанции
	u32 n=0;

		invc->Receiver_id =s->SENDER_ID[INDEX];//устанавливаем адрес получателя квитанции 
	
		n=invc->MSG.Num_cmd_in_msg;		//считываем число уже заполненных квитанций

	if (n<quantity_CMD)					//проверяем что есть место под очередную квитанцию
	{
		invc->MSG.Num_cmd_in_msg++;			  		    //увеличиваем счётчик заполненных квитанций
		invc->MSG.CMD[n].Cmd_time   =time;				//записываем текущее системное время
		invc->MSG.CMD[n].Cmd_type   =MSG_TYPE;			//тип сообщения 
		invc->MSG.CMD[n].Cmd_id	    =s->CMD_ID[INDEX];	//ID сообщения, указываем команду CMD к которой привязано сообщение.
		invc->MSG.CMD[n].Cmd_size   =1;					//размер данный квитанции
		invc->MSG.CMD[n].Cmd_data[0]=DATA_MSG;			//данные сообщения
		
		invc->MSG.Msg_size=(invc->MSG.Msg_size+
											16+			//размер Message header в байтах
											24+			//размер Command header в байтах
											1			//размер Cmd data
											);
		
//		u_out("Cmd_time:",time);
//		u_out("Cmd_type:",MSG_TYPE);
//		u_out("Cmd_id  :",s->CMD_ID[INDEX]);
//		u_out("Cmd_data:",DATA_MSG);
	
	} else 
	{
		STATUS=1;//возвращаем ошибку заполнения квитанции из-за отсутствия места
		Transf("нет места для квитанций!\r\n");
	}

//	u_out("число заполненных квитанций:",n);
	return STATUS;
}

//эта функция обслуживает создание квитанций состояния
u32 SYS_CMD_MSG(
ID_SERVER *s,//реестр
Frame *invc, //структура квитанций	
u32 INDEX,	 //индекс в реестре
u32 MSG_TYPE,//тип сообщения
u32 N,		 //объём данных сообщения в байтах
u8 *DATA_MSG,//данные сообщения - массив данных
u32 time	  //время составления квитанции
)
{
//	Transf("Заполняем квитанцию!\r\n");
	u32 STATUS=0;//успешно выполнили заполнение квитанции
	u32 n=0;
	u32 i=0;

		invc->Receiver_id =s->SENDER_ID[INDEX];//устанавливаем адрес получателя квитанции 
	
		n=invc->MSG.Num_cmd_in_msg;		//считываем число уже заполненных квитанций

	if (n<quantity_CMD)					//проверяем что есть место под очередную квитанцию
	{
		invc->MSG.Num_cmd_in_msg++;			  		    //увеличиваем счётчик заполненных квитанций
		invc->MSG.CMD[n].Cmd_time   =time;				//записываем текущее системное время
		invc->MSG.CMD[n].Cmd_type   =MSG_TYPE;			//тип сообщения 
		invc->MSG.CMD[n].Cmd_id	    =ID_CMD;			//ID сообщения, указываем команду CMD к которой привязано сообщение.
		invc->MSG.CMD[n].Cmd_size   =N;					//размер данный квитанции
		for (i=0;i<N;i++)
		{
			invc->MSG.CMD[n].Cmd_data[i]=DATA_MSG[i];   //данные сообщения
		}		
		
		invc->MSG.Msg_size=(invc->MSG.Msg_size+
											16+			//размер Message header в байтах
											24+			//размер Command header в байтах
											N			//размер Cmd data
											);
		
		ID_CMD++;
//		u_out("Cmd_time:",time);
//		u_out("Cmd_type:",MSG_TYPE);
//		u_out("Cmd_id  :",s->CMD_ID[INDEX]);
//		u_out("Cmd_data:",DATA_MSG);
	
	} else 
	{
		STATUS=1;//возвращаем ошибку заполнения квитанции из-за отсутствия места
		Transf("нет места для квитанций!\r\n");
	}

//	u_out("число заполненных квитанций:",n);
	return STATUS;
}

u32 SEND_UDP_MSG (void)
{
	u32 error=0;
	u32 i=0;
	for (i=0;i<quantity_SENDER;i++)//проверяем по адресатам нет ли квитанций кому?
	{
	  if (INVOICE[i].MSG.Num_cmd_in_msg>0) //!!! пока шлём все квитанции одному абаненту , несмотря на то что собираем их по всем по отдельности!!!
	  {
		 error=TX_MSG_BUFF (&INVOICE[i],TX_BUF,TX_MAX_BUF_SIZE); //заполняем транспортный массив
		 SEND_udp (destport_UDP,destip_UDP,TX_BUF,TX_BUF_LENGTH);//отправляем квитанцию по UDP
	  }
	}
	return error;
}

u32 Msg_uniq_id=0;//уникальный ID отправленных квитанций

u32 TX_MSG_BUFF (Frame *invc,uint8 *buf,u32 buf_size )
{
	u32 i=0;
	u32 j=0;
	u32 offset=0;
    u32 step=0;
	u32 error=0; 	
	
	Msg_uniq_id++;
	invc->Frame_number=0;
	invc->Msg_uniq_id =Msg_uniq_id;
	invc->Receiver_id =0;
	invc->Frame_size  =invc->MSG.Msg_size + 24;	
	
	buf[0]=(invc->Frame_size)>>8;
	buf[1]=(invc->Frame_size)   ;
	
	buf[2]=(invc->Frame_number)>>8;
	buf[3]=(invc->Frame_number|0x01);//установлен STOP BIT
	
	buf[4]=(invc->Msg_uniq_id)>>24;
	buf[5]=(invc->Msg_uniq_id)>>16;
	buf[6]=(invc->Msg_uniq_id)>> 8;
	buf[7]=(invc->Msg_uniq_id);
	
	buf[ 8]=(invc->Sender_id)>>56;
	buf[ 9]=(invc->Sender_id)>>48;
	buf[10]=(invc->Sender_id)>>40;
	buf[11]=(invc->Sender_id)>>32;
	buf[12]=(invc->Sender_id)>>24;
	buf[13]=(invc->Sender_id)>>16;
	buf[14]=(invc->Sender_id)>> 8;
	buf[15]=(invc->Sender_id);
	
	buf[16]=(invc->Receiver_id)>>56;
	buf[17]=(invc->Receiver_id)>>48;
	buf[18]=(invc->Receiver_id)>>40;
	buf[19]=(invc->Receiver_id)>>32;
	buf[20]=(invc->Receiver_id)>>24;
	buf[21]=(invc->Receiver_id)>>16;
	buf[22]=(invc->Receiver_id)>> 8;
	buf[23]=(invc->Receiver_id);
	
	buf[24]=(invc->MSG.Msg_size)>>24;
	buf[25]=(invc->MSG.Msg_size)>>16;
	buf[26]=(invc->MSG.Msg_size)>> 8;
	buf[27]=(invc->MSG.Msg_size);
	
	buf[28]=(invc->MSG.Msg_type)>>24;
	buf[29]=(invc->MSG.Msg_type)>>16;
	buf[30]=(invc->MSG.Msg_type)>> 8;
	buf[31]=(invc->MSG.Msg_type);
	
	buf[32]=(invc->MSG.Num_cmd_in_msg)>>56;
	buf[33]=(invc->MSG.Num_cmd_in_msg)>>48;
	buf[34]=(invc->MSG.Num_cmd_in_msg)>>40;
	buf[35]=(invc->MSG.Num_cmd_in_msg)>>32;
	buf[36]=(invc->MSG.Num_cmd_in_msg)>>24;
	buf[37]=(invc->MSG.Num_cmd_in_msg)>>16;
	buf[38]=(invc->MSG.Num_cmd_in_msg)>> 8;
	buf[39]=(invc->MSG.Num_cmd_in_msg);
	
	offset=40;
	for (i=0;i<invc->MSG.Num_cmd_in_msg;i++)
	{
		buf[offset+0]=(invc->MSG.CMD[i].Cmd_size)>>24;
		buf[offset+1]=(invc->MSG.CMD[i].Cmd_size)>>16;
		buf[offset+2]=(invc->MSG.CMD[i].Cmd_size)>> 8;
		buf[offset+3]=(invc->MSG.CMD[i].Cmd_size);
		
		buf[offset+4]=(invc->MSG.CMD[i].Cmd_type)>>24;
		buf[offset+5]=(invc->MSG.CMD[i].Cmd_type)>>16;
		buf[offset+6]=(invc->MSG.CMD[i].Cmd_type)>> 8;
		buf[offset+7]=(invc->MSG.CMD[i].Cmd_type);
		
		buf[offset+ 8]=(invc->MSG.CMD[i].Cmd_id)>>56;
		buf[offset+ 9]=(invc->MSG.CMD[i].Cmd_id)>>48;
		buf[offset+10]=(invc->MSG.CMD[i].Cmd_id)>>40;
		buf[offset+11]=(invc->MSG.CMD[i].Cmd_id)>>32;
		buf[offset+12]=(invc->MSG.CMD[i].Cmd_id)>>24;
		buf[offset+13]=(invc->MSG.CMD[i].Cmd_id)>>16;
		buf[offset+14]=(invc->MSG.CMD[i].Cmd_id)>> 8;
		buf[offset+15]=(invc->MSG.CMD[i].Cmd_id);
		
		buf[offset+16]=(invc->MSG.CMD[i].Cmd_time)>>56;
		buf[offset+17]=(invc->MSG.CMD[i].Cmd_time)>>48;
		buf[offset+18]=(invc->MSG.CMD[i].Cmd_time)>>40;
		buf[offset+19]=(invc->MSG.CMD[i].Cmd_time)>>32;
		buf[offset+20]=(invc->MSG.CMD[i].Cmd_time)>>24;
		buf[offset+21]=(invc->MSG.CMD[i].Cmd_time)>>16;
		buf[offset+22]=(invc->MSG.CMD[i].Cmd_time)>> 8;
		buf[offset+23]=(invc->MSG.CMD[i].Cmd_time);	

		for (j=0;j<invc->MSG.CMD[i].Cmd_size;j++) 
		{
			if (step<(buf_size-1)) step=offset+24+j; 
			else 	  
			{
				u_out("buf_size     :",buf_size);
				step=offset+24+j; 
				u_out("step         :",step);
				error=1;
				break;
			}//проверка переполнения массива
			buf[step]=invc->MSG.CMD[i].Cmd_data[j];
		}
		    if (offset<(buf_size-1-24-j)) offset=offset+24+j; 
			else
			{
				u_out("buf_size     :",buf_size);
				offset=offset+24+j;
				u_out("offset       :",offset);
				error=1;
				break;
			}//проверка переполнения массива
	}
	
	invc->MSG.Num_cmd_in_msg=0;//сбрасываем счётчик числа сообщений в структуре так как готовы её отослать
	
	TX_BUF_LENGTH=invc->Frame_size;//размер отправляемого массива
//	u_out("TX_BUF_LENGTH:",TX_BUF_LENGTH);
	
	invc->Frame_size=0;
	invc->MSG.Msg_size=0;
	invc->MSG.Num_cmd_in_msg=0;
	
	if (error>0) Transf ("переполнение транспортного буфера!\r\n");
	return error;
}

u32 time_return(void) 
{
  return TIME_SYS;
}

