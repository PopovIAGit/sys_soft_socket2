#include "http_server.h"
#include "stdint.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "string.h"
#include "stdbool.h"
#include "fs.h"
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include <stdint.h>
#include "cJSON.h"
#include "eeprom.h"
#include "process.h"
#include "analog.h"
#include "main.h"



static void CreateHTTPHeaderForPostResponse(char *answer){
        sprintf(answer, "%s", "HTTP/1.1 200 OK");
}

static void CreateHTTPResponse(char *answer, char *content_type, uint32_t length, char *content){
    
    if(length != 0){
        sprintf(answer, "%s%s%s%s%s%s%s%d%s%s", "HTTP/1.1 200 OK\r\n", 
                                    "Server: lwIP/1.3.1 (http://savannah.nongnu.org/projects/lwip)\r\n",            
                                    "Connection: keep-alive\r\n",
                                    "Content-Type: ", content_type, "\r\n",
                                    "Content-Length: ", length,
                                    "\r\n\r\n",
                                    content
                                    );
    } else {
        sprintf(answer, "%s%s%s%s%s", "HTTP/1.1 200 OK\r\n", 
                                    "Server: lwIP/1.3.1 (http://savannah.nongnu.org/projects/lwip)\r\n",            
                                    "Connection: keep-alive\r\n",
                                    "Content-Type: ", content_type);        
    }
}

static bool IsAuth = false;

static bool ParseHTTPPostValuesTextType(char* uri, char * name, char *value){
    char* ptr = strstr(uri, "\r\n\r\n");
    char *ptr_next;
    
    if(ptr == NULL) return false;
    ptr += 4;
    
    /*find name*/
    ptr = strstr(uri, name);
    if(ptr != NULL){
        ptr += strlen(name);
        /*find & or NULL*/
        ptr_next = strchr(ptr, '&');
        if(ptr_next != NULL){
            memcpy(value, ptr, (uint32_t)ptr_next - (uint32_t)ptr);
            value[(uint32_t)ptr_next - (uint32_t)ptr] = 0x00;
            return true;
        } else {
            strcpy(value, ptr);            
            return true;
        }
    }
    return false;
}

static bool ParseHTTPGetValuesTextType(char* uri, char * name, char *value, uint32_t len){
    const int symbol_1 = 0x3f; /*'?'*/
    const int symbol_2 = 0x26; /*'&'*/
    const int symbol_3 = 0x20; /*' '*/    
    uint32_t remaining_part;

    /*find end of map*/
    char *ptr_endurl = strstr(uri, " HTTP/1.1\r\n");
    
    /*find begining*/
    char* ptr_start = strchr(uri, symbol_1);    
    if(ptr_start == NULL) return false;
    ptr_start += 1;
    if((uint32_t)ptr_start >= (uint32_t)ptr_endurl) return false;    
    
    /*find name*/
    remaining_part = len - ((uint32_t)ptr_start - (uint32_t)uri);    
    ptr_start = strstr(ptr_start, name);
    if(ptr_start == NULL) return false;
    
    /*move start pointer to name length*/
    ptr_start = (void*)((uint32_t )ptr_start + strlen(name));
    if((uint32_t)ptr_start > (uint32_t)ptr_endurl) return false;    
    
    remaining_part = len - ((uint32_t)ptr_start - (uint32_t)uri);
    
    /*find map delimiter*/
    char* ptr_delimiter = strchr(ptr_start, symbol_2);
    char* ptr_end = ptr_endurl;//strchr(ptr_start, symbol_3);
    if((uint32_t)ptr_delimiter > (uint32_t)ptr_endurl) ptr_delimiter = NULL;        
    //if((uint32_t)ptr_end >= (uint32_t)ptr_endurl) ptr_end = NULL;        

    
    if(ptr_delimiter != NULL){
        ptr_end = (void*)((uint32_t )ptr_delimiter);
    }
    
    //if(ptr_end == NULL) return false;
    char val_size = (uint32_t)ptr_end - (uint32_t)ptr_start;
    memset(value, 0x00, val_size + 1);
    strncpy(value, ptr_start, val_size); 
    
    return true;
}

void http_server(){
    static int listen_sock, peer_sock, len, new_socket,err;
    const int keepIdle = 10;
    const int keepInterval = 10;
    const int keepCount = 3;
    static int opt = 1;
    static char jbuffer[2048];
    static struct sockaddr_in server_addr = {0};
    static struct pollfd fds;
    static struct fs_file file;
    static struct sockaddr peeraddr = {0};
    static socklen_t peeraddr_size = 14;
    static char value[32];
    static char web_page[1300];
    static devparam_t devparam;
    static cJSON *json, *mean; 
    static uint32_t length;
    static uint8_t mac_data[6], dev_data[4], mask_data[4], gw_data[4];
    static ip_addr_t ip;
    static float zero[7];
    static float full[10]; 
    static uint16_t suz, rscurr, freq, coilcurr, maxRs, maxHs, maxDispersion;
    enum {
        r_full_0 = 30,
        r_full_1 = 70,
        r_full_2 = 200,
        r_full_3 = 400,
        r_full_4 = 900,
        r_full_5 = 2000,
        r_full_6 = 4000
    };
    
    /*create tcp socket*/
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    
    if (listen_sock == -1) {
        __disable_interrupt();
        while (1);
    }
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(80);

    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, & opt, sizeof(opt));
    bind(listen_sock, (struct sockaddr * ) &server_addr, sizeof(server_addr));
    if (listen(listen_sock, 1) == -1) {
        __disable_interrupt();
        while (1);
    }

    IsAuth = true;	        
    
    for(;;){
        /*create sockets for polling*/
        fds.fd = listen_sock;
        fds.events  = POLLIN;
        fds.revents = 0;
        
        if(poll(&fds, 1, -1) == -1){
            __disable_interrupt();
            while(1);
        }
        
        if(fds.revents & POLLIN){
            new_socket = accept(listen_sock, (struct sockaddr * ) &peeraddr, &peeraddr_size);            
            //close(peer_sock);  
            //shutdown(peer_sock, SHUT_RDWR); 
            //osDelay(100);
            peer_sock = new_socket;
            opt = 1;
            setsockopt(peer_sock, SOL_SOCKET, SO_KEEPALIVE, & opt, sizeof(opt));
            setsockopt(peer_sock, IPPROTO_TCP, TCP_KEEPIDLE, & keepIdle, sizeof(int));
            setsockopt(peer_sock, IPPROTO_TCP, TCP_KEEPINTVL, & keepInterval, sizeof(int));
            setsockopt(peer_sock, IPPROTO_TCP, TCP_KEEPCNT, & keepCount, sizeof(int));                                
        }
        
        fds.fd = peer_sock;
        fds.events  = POLLIN|POLLERR;
        fds.revents = 0;
        
        if(poll(&fds, 1, -1) == -1){
            __disable_interrupt();
            while(1);
        }        
        
        if(fds.revents & POLLERR){
            close(peer_sock);  
            shutdown(peer_sock, SHUT_RDWR);            
            continue;
        }
        
        memset(jbuffer, 0x00, sizeof(jbuffer));
        len = recv(peer_sock, (uint8_t * ) jbuffer, sizeof(jbuffer), 0);
        if(len > 0){/*parse http*/
            if(strstr((char const *)jbuffer, "GET")){
                if ((strstr((char const *)jbuffer, " / ") != NULL)||(strstr((char const *)jbuffer, " /index.shtml") != NULL)){
                    if(ParseHTTPGetValuesTextType(jbuffer, "pass=", value, len)){
                        if(strcmp(value, "admin") == 0){
                            fs_open(&file, "/main_conf.shtml");
                            send(peer_sock, file.data, file.len, 0);  
                            fs_close(&file);                  
                        } else {
                            fs_open(&file, "/index.shtml");
                            err = send(peer_sock, file.data, file.len, 0);
                            fs_close(&file);
                        }
                    } else {
                        fs_open(&file, "/index.shtml");
                        err = send(peer_sock, file.data, file.len, 0);
                        fs_close(&file);                        
                    }
                    goto close_conn;  
                }
                
                if (strstr((char const *)jbuffer, " /config_eth.shtml") != NULL){
                    if(strstr((char const *)jbuffer, "/parametres") != NULL){
                        memset(web_page, 0x00, sizeof(web_page));
                        json = cJSON_CreateObject();
                        if(json == NULL) return;
                        ModuleGetParam(&devparam);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.mac));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "macaddr0", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.mac >> 8));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "macaddr1", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.mac >> 16));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "macaddr2", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.mac >> 24));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "macaddr3", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.mac >> 32));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "macaddr4", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.mac >> 40));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "macaddr5", mean);

                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.dev_ip.addr));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "dev_ip_ad0", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.dev_ip.addr >> 8));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "dev_ip_ad1", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.dev_ip.addr >> 16));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "dev_ip_ad2", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.dev_ip.addr >> 24));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "dev_ip_ad3", mean);

                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.gw_ip.addr));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "gw_ip_ad0", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.gw_ip.addr >> 8));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "gw_ip_ad1", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.gw_ip.addr >> 16));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "gw_ip_ad2", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.gw_ip.addr >> 24));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "gw_ip_ad3", mean);                              

                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.netmask.addr));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "mk_ip_ad0", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.netmask.addr >> 8));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "mk_ip_ad1", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.netmask.addr >> 16));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "mk_ip_ad2", mean);
                        mean = cJSON_CreateNumber((uint8_t) (devparam.eth.netmask.addr >> 24));
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "mk_ip_ad3", mean); 
                        cJSON_PrintPreallocated(json, web_page, sizeof(web_page), cJSON_True);
                        cJSON_Delete(json);

                        length = strlen(web_page);                        
                        memset(jbuffer, 0x00, sizeof(jbuffer));
                        CreateHTTPResponse(jbuffer, "application/json", length, web_page);                        
                        send(peer_sock, jbuffer, strlen(jbuffer), 0);
                    } else {                         
                         if(ParseHTTPGetValuesTextType(jbuffer, "macaddr0=", value, len)) mac_data[0] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "macaddr1=", value, len)) mac_data[1] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "macaddr2=", value, len)) mac_data[2] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "macaddr3=", value, len)) mac_data[3] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "macaddr4=", value, len)) mac_data[4] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "macaddr5=", value, len)) mac_data[5] = atoi(value); 

                         if(ParseHTTPGetValuesTextType(jbuffer, "dev_ip_ad0=", value, len)) dev_data[0] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "dev_ip_ad1=", value, len)) dev_data[1] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "dev_ip_ad2=", value, len)) dev_data[2] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "dev_ip_ad3=", value, len)) dev_data[3] = atoi(value); 
                        
                         if(ParseHTTPGetValuesTextType(jbuffer, "mk_ip_ad0=", value, len)) mask_data[0] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "mk_ip_ad1=", value, len)) mask_data[1] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "mk_ip_ad2=", value, len)) mask_data[2] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "mk_ip_ad3=", value, len)) mask_data[3] = atoi(value); 
 
                         if(ParseHTTPGetValuesTextType(jbuffer, "gw_ip_ad0=", value, len)) gw_data[0] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "gw_ip_ad1=", value, len)) gw_data[1] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "gw_ip_ad2=", value, len)) gw_data[2] = atoi(value); 
                         if(ParseHTTPGetValuesTextType(jbuffer, "gw_ip_ad3=", value, len)) gw_data[3] = atoi(value);                          
                         
                         if(ParseHTTPGetValuesTextType(jbuffer, "eth_sip=", value, len)){
                            ModuleGetParam(&devparam);
                            IP4_ADDR(&ip, dev_data[0], dev_data[1], dev_data[2], dev_data[3]);
                            devparam.eth.dev_ip.addr = ip.addr;         
                            ModuleSetParam(&devparam);
                           } 
                         
                         if(ParseHTTPGetValuesTextType(jbuffer, "eth_sgw=", value, len)){
                            ModuleGetParam(&devparam);
                            IP4_ADDR(&ip, gw_data[0], gw_data[1], gw_data[2], gw_data[3]);
                            devparam.eth.gw_ip.addr = ip.addr;         
                            ModuleSetParam(&devparam);
                           } 
                         
                         if(ParseHTTPGetValuesTextType(jbuffer, "eth_smk=", value, len)){
                            ModuleGetParam(&devparam);
                            IP4_ADDR(&ip, mask_data[0], mask_data[1], mask_data[2], mask_data[3]);
                            devparam.eth.netmask.addr = ip.addr;         
                            ModuleSetParam(&devparam);
                           } 
                         
                         if(ParseHTTPGetValuesTextType(jbuffer, "eth_smac=", value, len)){
                            ModuleGetParam(&devparam);
                            devparam.eth.mac = (uint64_t)mac_data[0] | ((uint64_t)mac_data[1] << 8)  | ((uint64_t)mac_data[2] << 16) | 
                                                ((uint64_t)mac_data[3] << 24) | ((uint64_t)mac_data[4] << 32) | ((uint64_t)mac_data[5] << 40);          
                            ModuleSetParam(&devparam);
                         }
                        fs_open(&file, "/config_eth.shtml");
                        send(peer_sock, file.data, file.len, 0);  
                        fs_close(&file);   
                    }
                    goto close_conn; 
                }
                
                if ((strstr((char const *)jbuffer, " /main_conf.shtml") != NULL) && IsAuth){
                    if(strstr((char const *)jbuffer, "/parametres") != NULL){
                        memset(web_page, 0x00, sizeof(web_page));
                        json = cJSON_CreateObject();
                        if(json == NULL) return;
                        ModuleGetParam(&devparam);
			
                        mean = cJSON_CreateNumber(GetLastRSon());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "RSon", mean);
			
                        mean = cJSON_CreateNumber(GetRS_OFF());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "RSoff", mean);
			
                        mean = cJSON_CreateNumber(GetDispertion());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "Dispersion", mean);
			
                        mean = cJSON_CreateNumber(GetSavedRIso());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "Rstat", mean);
                      //  mean = cJSON_CreateNumber(GetB_HS());
			mean = cJSON_CreateNumber(GetCalibHS());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "Field", mean);
                        mean = cJSON_CreateNumber(GetRSCurrent());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "Iwork", mean);

                        mean = cJSON_CreateNumber(/*GetCoilPWMFreq()*/ devparam.testparam.Fcoil);
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "Freq", mean);
			
                        mean = cJSON_CreateNumber(GetCoilCurrent());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "Icoil", mean);
			
                        mean = cJSON_CreateNumber(GetTransientTime());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "Time", mean);
			
                        mean = cJSON_CreateNumber(GetSuzType());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "SUJType", mean);

                        mean = cJSON_CreateString(VERSION);
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "Version", mean);
                        
                        mean = cJSON_CreateNumber(devparam.testparam.MaxRson);
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "MaxRs", mean);
                       
			mean = cJSON_CreateNumber(devparam.testparam.MaxDispersion);
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "MaxDispertion", mean);                        
                        
			mean = cJSON_CreateNumber(devparam.testparam.MaxHS);
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "MaxHs", mean); 
			
			mean = cJSON_CreateNumber(GetStatus());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "Status", mean); 
			
                        mean = cJSON_CreateNumber(GetConnectStatus());
                        if(mean == NULL) return;
                        cJSON_AddItemToObject(json, "Connect", mean);
			
                        cJSON_PrintPreallocated(json, web_page, sizeof(web_page), cJSON_True);
                        cJSON_Delete(json);

                        length = strlen(web_page);                        
                        memset(jbuffer, 0x00, sizeof(jbuffer));
                        CreateHTTPResponse(jbuffer, "application/json", length, web_page);                        
                        send(peer_sock, jbuffer, strlen(jbuffer), 0);
                    } else {
                        if(ParseHTTPGetValuesTextType(jbuffer, "freq=", value, len)) freq = atoi(value);  
                        if(ParseHTTPGetValuesTextType(jbuffer, "rscurr=", value, len)) rscurr = atoi(value);
                        if(ParseHTTPGetValuesTextType(jbuffer, "suz=", value, length)) suz = atoi(value);
                        if(ParseHTTPGetValuesTextType(jbuffer, "coilcurr=", value, len)) coilcurr = atoi(value);
 
                        if(ParseHTTPGetValuesTextType(jbuffer, "maxRs=", value, len)) maxRs = atoi(value);
                        if(ParseHTTPGetValuesTextType(jbuffer, "maxDispersion=", value, len)) maxDispersion = atoi(value); 
                        if(ParseHTTPGetValuesTextType(jbuffer, "maxHs=", value, len)) maxHs = atoi(value); 
                        
                        if(ParseHTTPGetValuesTextType(jbuffer, "cyc_tststart=", value, len)){
                            SetCycleTestMode();
                            StartRSTest();
			    SetStatus(st_CYCLEMODE);
                        }
                        if(ParseHTTPGetValuesTextType(jbuffer, "cyc_tststop=", value, len)){
                            StopRSTest(); 
                            // SetSignleTestMode();
			     SetStatus(st_READY);
                        }      
                        if(ParseHTTPGetValuesTextType(jbuffer, "single_tststart=", value, len)){
                           if (GetSuzType()==0) // если суж “ќћ«ЁЋ
			   {
			      SetOnContinuously(); 
                              SetFindR(true);
			      SetStatus(st_SIGNEMODE);
			   }
			   else { // если все остальные сужи
				SetSignleTestMode();
				StartRSTest();
				SetStatus(st_SIGNEMODE);
			   }

                        }
			if(ParseHTTPGetValuesTextType(jbuffer, "mag_tststart=", value, len)){
				StartMagnitTest();  
				SetStatus(st_FLOAT_TEST);
			} 
			// запуск поиска поплавка и геркона
			if(ParseHTTPGetValuesTextType(jbuffer, "mag_serch_down=", value, len)){
			  
			  SetPoplovokCalib();
  
			    SetStatus(st_POPLOVOK_CALIB);
                        }
			if(ParseHTTPGetValuesTextType(jbuffer, "mag_serch_up=", value, len)){
                 
			 	 SetPoplovokCalib();
			    SetStatus(st_POPLOVOK_CALIB);
                        }
			if(ParseHTTPGetValuesTextType(jbuffer, "tst_serch=", value, len)){
      
			    
			    if (GetSuzType()==0) // если суж “ќћ«ЁЋ
			   {
			      SetOnContinuously(); 
                              SetFindMinR(true);
			      SetStatus(st_FIND_GERKON_MODUL);
			   }
			   else { // если все остальные сужи
				SetCycleTestMode();
	  			StartRSTest();
				SetFindMinR(true);
				SetStatus(st_FIND_GERKON);
			   }
                        }
			  
                        if(ParseHTTPGetValuesTextType(jbuffer, "s_rscurr=", value, len)){
                            SetRSCurrent(rscurr); 
                            ModuleGetParam(&devparam);
                            devparam.testparam.Irs = rscurr;          
                            ModuleSetParam(&devparam);     
                        }
                        if(ParseHTTPGetValuesTextType(jbuffer, "s_suz=", value, len)){
                           SetSuzType(suz); 
                           ModuleGetParam(&devparam);
                           devparam.testparam.Type = suz;          
                           ModuleSetParam(&devparam);     
                        }
                        if(ParseHTTPGetValuesTextType(jbuffer, "s_coilcurr=", value, len)){
                            SetCoilCurrent(coilcurr);   
                            ModuleGetParam(&devparam);
                            devparam.testparam.Icoil = coilcurr;          
                            ModuleSetParam(&devparam);          
                         }
                        if(ParseHTTPGetValuesTextType(jbuffer, "iso_tststart=", value, len)) StartRISOTest(); 
                        if(ParseHTTPGetValuesTextType(jbuffer, "sfreq=", value, len)){
                            SetCoilPWMFreq(freq);    
                            ModuleGetParam(&devparam);
                            devparam.testparam.Fcoil = freq;          
                            ModuleSetParam(&devparam);               
                        }
                        if(ParseHTTPGetValuesTextType(jbuffer, "coilon=", value, len)){              
                            //StartRSTest();
                            SetOnContinuously(); 
			    SetStatus(st_permanently);
                        }                        
                        if(ParseHTTPGetValuesTextType(jbuffer, "s_maxRs=", value, len)){             
                            SetMaxRs(maxRs);  
                            ModuleGetParam(&devparam);
                            devparam.testparam.MaxRson = maxRs;          
                            ModuleSetParam(&devparam);          
                        }
                        if(ParseHTTPGetValuesTextType(jbuffer, "s_maxDispersion=", value, len)){     
                            SetMaxDispersion(maxDispersion); 
                            ModuleGetParam(&devparam);
                            devparam.testparam.MaxDispersion = maxDispersion;          
                            ModuleSetParam(&devparam);          
                        }
                        if(ParseHTTPGetValuesTextType(jbuffer, "s_maxHs=", value, len)){             
                            SetMaxHs(maxHs);  
                            ModuleGetParam(&devparam);
                            devparam.testparam.MaxHS = maxHs;          
                            ModuleSetParam(&devparam);          
                        }
                        fs_open(&file, "/main_conf.shtml");
                        write(peer_sock, file.data, file.len);  
                        fs_close(&file);              
                    }                    
                    goto close_conn; 
                }
                
                if ((strstr((char const *)jbuffer, " /calibrate.shtml") != NULL) && IsAuth){
                    if(ParseHTTPGetValuesTextType(jbuffer, "k1_offset=", value, len)) zero[0] = Calibration(GAIN_100);
		    if(ParseHTTPGetValuesTextType(jbuffer, "k2_offset=", value, len)) zero[1] = Calibration(GAIN_50);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k3_offset=", value, len)) zero[2] = Calibration(GAIN_20);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k4_offset=", value, len)) zero[3] = Calibration(GAIN_10);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k5_offset=", value, len)) zero[4] = Calibration(GAIN_5);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k6_offset=", value, len)) zero[5] = Calibration(GAIN_2);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k7_offset=", value, len)) zero[6] = Calibration(GAIN_1);
		    
                    if(ParseHTTPGetValuesTextType(jbuffer, "k1_gain=", value, len)) full[0] = Calibration(GAIN_100);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k2_gain=", value, len)) full[1] = Calibration(GAIN_50);
		    if(ParseHTTPGetValuesTextType(jbuffer, "k3_gain=", value, len)) full[2] = Calibration(GAIN_20);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k4_gain=", value, len)) full[3] = Calibration(GAIN_10);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k5_gain=", value, len)) full[4] = Calibration(GAIN_5);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k6_gain=", value, len)) full[5] = Calibration(GAIN_2);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k7_gain=", value, len)) full[6] = Calibration(GAIN_1);
		    
		    if(ParseHTTPGetValuesTextType(jbuffer, "k9_gain=",  value, len)) full[7] = CalibRIso(GAIN_2);
                    if(ParseHTTPGetValuesTextType(jbuffer, "k10_gain=", value, len)) full[8] = CalibRIso(GAIN_1);
		    if(ParseHTTPGetValuesTextType(jbuffer, "k11_gain=", value, len)) full[9] = CalibRIso(GAIN_0);
                            
                    if(ParseHTTPGetValuesTextType(jbuffer, "k1_save=", value, len)){
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.R_gain_GAIN_100 = (-r_full_0)/(zero[0]-full[0]);
                        devparam.calibrate.R_offset_GAIN_100 = -devparam.calibrate.R_gain_GAIN_100*zero[0];
                        ModuleSetParam(&devparam); 
                    }
                    if(ParseHTTPGetValuesTextType(jbuffer, "k2_save=", value, len)) {
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.R_gain_GAIN_50 = (-r_full_1)/(zero[1]-full[1]);
                        devparam.calibrate.R_offset_GAIN_50 = -devparam.calibrate.R_gain_GAIN_50*zero[1];
                        ModuleSetParam(&devparam); 
                   }
                   if(ParseHTTPGetValuesTextType(jbuffer, "k3_save=", value, len)) {
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.R_gain_GAIN_20 = (-r_full_2)/(zero[2]-full[2]);
                        devparam.calibrate.R_offset_GAIN_20 = -devparam.calibrate.R_gain_GAIN_20*zero[2];
                        ModuleSetParam(&devparam); 
                   }
                   if(ParseHTTPGetValuesTextType(jbuffer, "k4_save=", value, len)) {
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.R_gain_GAIN_10 = (-r_full_3)/(zero[3]-full[3]);
                        devparam.calibrate.R_offset_GAIN_10 = -devparam.calibrate.R_gain_GAIN_10*zero[3];
                        ModuleSetParam(&devparam); 
                   }
                   if(ParseHTTPGetValuesTextType(jbuffer, "k5_save=", value, len)) {
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.R_gain_GAIN_5 = (-r_full_4)/(zero[4]-full[4]);
                        devparam.calibrate.R_offset_GAIN_5 = -devparam.calibrate.R_gain_GAIN_5*zero[4];
                        ModuleSetParam(&devparam); 
                   }
                   if(ParseHTTPGetValuesTextType(jbuffer, "k6_save=", value, len)) {
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.R_gain_GAIN_2 = (-r_full_5)/(zero[5]-full[5]);
                        devparam.calibrate.R_offset_GAIN_2 = -devparam.calibrate.R_gain_GAIN_2*zero[5];
                        ModuleSetParam(&devparam); 
                   }
                   if(ParseHTTPGetValuesTextType(jbuffer, "k7_save=", value, len)) {
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.R_gain_GAIN_1 = (-r_full_6)/(zero[6]-full[6]);
                        devparam.calibrate.R_offset_GAIN_1 = -devparam.calibrate.R_gain_GAIN_1*zero[6];
                        ModuleSetParam(&devparam); 
                   }
                   if(ParseHTTPGetValuesTextType(jbuffer, "k8_offset=", value, len)) {
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.HS_offset = GetU_HS();
                        ModuleSetParam(&devparam); 
                   }
		   if(ParseHTTPGetValuesTextType(jbuffer, "k9_save=", value, len)) {
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.R_ONE_GERKON = full[7];
                        ModuleSetParam(&devparam); 
                   }  
		   if(ParseHTTPGetValuesTextType(jbuffer, "k10_save=", value, len)) {
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.R_TWO_GERKON = full[8];
                        ModuleSetParam(&devparam); 
                   }  
		   if(ParseHTTPGetValuesTextType(jbuffer, "k11_save=", value, len)) {
                        ModuleGetParam(&devparam);      
                        devparam.calibrate.R_THREE_GERKON = full[9];
                        ModuleSetParam(&devparam); 
                   }  

                   fs_open(&file, "/calibrate.shtml");
                   write(peer_sock, file.data, file.len);  
                   fs_close(&file);                                           
                   goto close_conn; 
                }
                
                if ((strstr((char const *)jbuffer, "/favicon.ico") != NULL)){
                    fs_open(&file, "/favicon.ico");
                    send(peer_sock, file.data, file.len, 0);  
                    fs_close(&file);   
                    goto close_conn; 
                }
                fs_open(&file, "/404.shtml");
                send(peer_sock, file.data, file.len, 0);  
                fs_close(&file);                 
            } else if(strstr((char const *)jbuffer, "POST")){
                
            }
close_conn :            
            close(peer_sock);  
            shutdown(peer_sock, SHUT_RDWR);
        }
        osThreadYield(); 		
    }
}

