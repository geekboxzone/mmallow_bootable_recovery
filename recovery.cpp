/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _U
#define _U _CTYPE_U
#endif
#ifndef _L
#define _L _CTYPE_L
#endif
#ifndef _N
#define _N _CTYPE_L
#endif
#ifndef _X
#define _X _CTYPE_X
#endif
#ifndef _P
#define _P _CTYPE_P
#endif
#ifndef _B
#define _B _CTYPE_B
#endif
#ifndef _C
#define _C _CTYPE_C
#endif
#ifndef _S
#define _S _CTYPE_S
#endif

#include <ctype.h>
#include <dirent.h>
#include <fs_mgr.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <linux/input.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/klog.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <sys/wait.h>
#include <unistd.h>
#include <sys/mount.h>
#include <base/file.h>
#include <base/stringprintf.h>
#include <dirent.h>
#include "bootloader.h"
#include "common.h"
#include "cutils/properties.h"
#include "cutils/android_reboot.h"
#include "install.h"
#include "minui/minui.h"
#include "minzip/DirUtil.h"
#include "roots.h"
#include "ui.h"
#include "screen_ui.h"
#include "device.h"
#include "adb_install.h"
#include "mtdutils/mounts.h"
#include "sdtool.h"
#include "adb.h"
extern "C" {
#include "rkimage.h"
#include "mtdutils/rk29.h"
#include "mtdutils/mtdutils.h"
#include "rkimage.h"
}
#include "fuse_sideload.h"
#include "fuse_sdcard_provider.h"

#include "recovery_utils.h"
#include "sdtool.h"
#include <pthread.h>
#include "rkupdate/Upgrade.h"
struct selabel_handle *sehandle;

static const struct option OPTIONS[] = {
  { "factory_mode", required_argument, NULL, 'f' },
  { "send_intent", required_argument, NULL, 'i' },
  { "update_package", required_argument, NULL, 'u' },
  { "update_rkimage", required_argument, NULL, 'k' },   // support rkimage to update
  { "wipe_data", no_argument, NULL, 'w' },
  { "wipe_cache", no_argument, NULL, 'c' },
  { "show_text", no_argument, NULL, 't' },
  { "sideload", no_argument, NULL, 's' },
  { "sideload_auto_reboot", no_argument, NULL, 'a' },
  { "just_exit", no_argument, NULL, 'x' },
  { "locale", required_argument, NULL, 'l' },
  { "stages", required_argument, NULL, 'g' },
  { "shutdown_after", no_argument, NULL, 'p' },
  { "fw_update", required_argument, NULL, 'f'+'w' },
  { "demo_copy", required_argument, NULL, 'd' },
  { "reason", required_argument, NULL, 'r' },
  { "wipe_all", no_argument, NULL, 'w'+'a' },
  { NULL, 0, NULL, 0 },
};

static const char *CACHE_LOG_DIR = "/cache/recovery";
static const char *COMMAND_FILE = "/cache/recovery/command";
static const char *FLAG_FILE = "/cache/recovery/last_flag";
static const char *INTENT_FILE = "/cache/recovery/intent";
static const char *LOG_FILE = "/cache/recovery/log";
static const char *LAST_INSTALL_FILE = "/cache/recovery/last_install";
static const char *LOCALE_FILE = "/cache/recovery/last_locale";
static const char *CACHE_ROOT = "/cache";
static const char *USB_ROOT = "/mnt/usb_storage";
static const char *SDCARD_ROOT = "/sdcard";
static const char *TEMPORARY_LOG_FILE = "/tmp/recovery.log";
static const char *TEMPORARY_INSTALL_FILE = "/tmp/last_install";
static char IN_SDCARD_ROOT[256] = "\0";
static char EX_SDCARD_ROOT[256] = "\0";
static char USB_DEVICE_PATH[128] = "\0";
static char updatepath[128] = "\0";
bool bNeedClearMisc=true;
bool bAutoUpdateComplete = false;
bool bFactoryMode = false;
static const char *LAST_KMSG_FILE = "/cache/recovery/last_kmsg";
#define KLOG_DEFAULT_LEN (64 * 1024)
static const char *LAST_LOG_FILE = "/cache/recovery/last_log";
static const int KEEP_LOG_COUNT = 10;

RecoveryUI* ui = NULL;
char* locale = NULL;
char* stage = NULL;
char* reason = NULL;
bool bIfUpdateLoader = false;
char gVolume_label[128];
enum ConfigId {
	pcba_test = 0,
	fw_update,
	display_lcd,
	display_led,
	demo_copy,
	volume_label
};
static RKSdBootCfgItem SdBootConfigs[] = 
{
	{ 
        (char*)"pcba_test",
        (char*)"1"
    },
	{ 
        (char*)"fw_update",
        (char*)"0"
    },
	{ 
        (char*)"display_lcd",
        (char*)"1"
    },
    { 
        (char*)"display_led",
        (char*)"1"
    },
	{ 
        (char*)"demo_copy",
        (char*)"0"
    },
	{ 
        (char*)"volume_label",
        (char*)"rockchip"
    },
};
pthread_t tid;
bool modified_flash = false;
bool bSDMounted = false;
bool bUsbMounted = false;

/*
 * The recovery tool communicates with the main system through /cache files.
 *   /cache/recovery/command - INPUT - command line for tool, one arg per line
 *   /cache/recovery/log - OUTPUT - combined log file from recovery run(s)
 *   /cache/recovery/intent - OUTPUT - intent that was passed in
 *
 * The arguments which may be supplied in the recovery.command file:
 *   --send_intent=anystring - write the text out to recovery.intent
 *   --update_package=path - verify install an OTA package file
 *   --wipe_data - erase user data (and cache), then reboot
 *   --wipe_cache - wipe cache (but not user data), then reboot
 *   --set_encrypted_filesystem=on|off - enables / diasables encrypted fs
 *   --just_exit - do nothing; exit and reboot
 *
 * After completing, we remove /cache/recovery/command and reboot.
 * Arguments may also be supplied in the bootloader control block (BCB).
 * These important scenarios must be safely restartable at any point:
 *
 * FACTORY RESET
 * 1. user selects "factory reset"
 * 2. main system writes "--wipe_data" to /cache/recovery/command
 * 3. main system reboots into recovery
 * 4. get_args() writes BCB with "boot-recovery" and "--wipe_data"
 *    -- after this, rebooting will restart the erase --
 * 5. erase_volume() reformats /data
 * 6. erase_volume() reformats /cache
 * 7. finish_recovery() erases BCB
 *    -- after this, rebooting will restart the main system --
 * 8. main() calls reboot() to boot main system
 *
 * OTA INSTALL
 * 1. main system downloads OTA package to /cache/some-filename.zip
 * 2. main system writes "--update_package=/cache/some-filename.zip"
 * 3. main system reboots into recovery
 * 4. get_args() writes BCB with "boot-recovery" and "--update_package=..."
 *    -- after this, rebooting will attempt to reinstall the update --
 * 5. install_package() attempts to install the update
 *    NOTE: the package install must itself be restartable from any point
 * 6. finish_recovery() erases BCB
 *    -- after this, rebooting will (try to) restart the main system --
 * 7. ** if install failed **
 *    7a. prompt_and_wait() shows an error icon and waits for the user
 *    7b; the user reboots (pulling the battery, etc) into the main system
 * 8. main() calls maybe_install_firmware_update()
 *    ** if the update contained radio/hboot firmware **:
 *    8a. m_i_f_u() writes BCB with "boot-recovery" and "--wipe_cache"
 *        -- after this, rebooting will reformat cache & restart main system --
 *    8b. m_i_f_u() writes firmware image into raw cache partition
 *    8c. m_i_f_u() writes BCB with "update-radio/hboot" and "--wipe_cache"
 *        -- after this, rebooting will attempt to reinstall firmware --
 *    8d. bootloader tries to flash firmware
 *    8e. bootloader writes BCB with "boot-recovery" (keeping "--wipe_cache")
 *        -- after this, rebooting will reformat cache & restart main system --
 *    8f. erase_volume() reformats /cache
 *    8g. finish_recovery() erases BCB
 *        -- after this, rebooting will (try to) restart the main system --
 * 9. main() calls reboot() to boot main system
 */

static const int MAX_ARG_LENGTH = 4096;
static const int MAX_ARGS = 100;

void handle_upgrade_callback(char *szPrompt)
{
    if (ui){
        ui->Print(szPrompt);
        ui->Print("\n");
    }
}
void handle_upgrade_progress_callback(float portion, float seconds)
{
    if (ui){
        if (seconds==0)
            ui->SetProgress(portion);
        else
            ui->ShowProgress(portion,seconds);
    }
}
// open a given path, mounting partitions as necessary
FILE*
fopen_path(const char *path, const char *mode) {
    if (ensure_path_mounted(path) != 0) {
        LOGE("Can't mount %s\n", path);
        return NULL;
    }

    // When writing, try to create the containing directory, if necessary.
    // Use generous permissions, the system (init.rc) will reset them.
    if (strchr("wa", mode[0])) dirCreateHierarchy(path, 0777, NULL, 1, sehandle);

    FILE *fp = fopen(path, mode);
    return fp;
}

bool is_ro_debuggable() {
    char value[PROPERTY_VALUE_MAX+1];
    return (property_get("ro.debuggable", value, NULL) == 1 && value[0] == '1');
}

void SetSdcardRootPath();
static void redirect_stdio(const char* filename) {
    // If these fail, there's not really anywhere to complain...
    freopen(filename, "a", stdout); setbuf(stdout, NULL);
    freopen(filename, "a", stderr); setbuf(stderr, NULL);
}

// close a file, log an error if the error indicator is set
static void
check_and_fclose(FILE *fp, const char *name) {
    fflush(fp);
    if (ferror(fp)) LOGE("Error in %s\n(%s)\n", name, strerror(errno));
    fclose(fp);
}

bool parse_config(char *pConfig,VEC_SD_CONFIG &vecItem)
{
    printf("in parse_config\n");
    std::stringstream configStream(pConfig);
    std::string strLine,strItemName,strItemValue;
    std::string::size_type line_size,pos;
    vecItem.clear();
    STRUCT_SD_CONFIG_ITEM item;
    while (!configStream.eof())
    {
        getline(configStream,strLine);
        line_size = strLine.size();
        if (line_size==0)
            continue;
        if (strLine[line_size-1]=='\r')
        {
            strLine = strLine.substr(0,line_size-1);
        }
        printf("%s\n",strLine.c_str());
        pos = strLine.find("=");
        if (pos==std::string::npos)
        {
            continue;
        }
        if (strLine[0]=='#')
        {
            continue;
        }
        strItemName = strLine.substr(0,pos);
        strItemValue = strLine.substr(pos+1);
        strItemName.erase(0,strItemName.find_first_not_of(" "));
        strItemName.erase(strItemName.find_last_not_of(" ")+1);
        strItemValue.erase(0,strItemValue.find_first_not_of(" "));
        strItemValue.erase(strItemValue.find_last_not_of(" ")+1);
        if ((strItemName.size()>0)&&(strItemValue.size()>0))
        {
            item.strKey = strItemName;
            item.strValue = strItemValue;
            vecItem.push_back(item);
        }
    }
    printf("out parse_config\n");
    return true;
    
}

bool parse_config_file(const char *pConfigFile,VEC_SD_CONFIG &vecItem)
{
    FILE *file=NULL;
    file = fopen(pConfigFile,"rb");
    if( !file ){
        return false;
    }
    int iFileSize;
    fseek(file,0,SEEK_END);
    iFileSize = ftell(file);
    fseek(file,0,SEEK_SET);
    char *pConfigBuf=NULL;
    pConfigBuf = new char[iFileSize+1];
    if (!pConfigBuf)
    {
        fclose(file);
        return false;
    }
    memset(pConfigBuf,0,iFileSize+1);
    int iRead;
    iRead = fread(pConfigBuf,1,iFileSize,file);
    if (iRead!=iFileSize)
    {
        fclose(file);
        delete []pConfigBuf;
        return false;
    }
    fclose(file);
    bool bRet;
    bRet = parse_config(pConfigBuf,vecItem);
    delete []pConfigBuf;
    printf("out parse_config_file\n");
    return bRet;
}

int mount_usb_device()
{
	char configFile[64];
	char usbDevice[64];
	int result;
	DIR* d=NULL;
	struct dirent* de;
	d = opendir(USB_ROOT);
	if (d)
	{//check whether usb_root has  mounted
		strcpy(configFile, USB_ROOT);
		strcat(configFile, "/sd_boot_config.config");
		if (access(configFile,F_OK)==0)
		{
			closedir(d);
			return 0;
		}
		closedir(d);
	}
	else
	{
		if (errno==ENOENT)
		{
			if (mkdir(USB_ROOT,0755)!=0)
		    {
				printf("failed to create %s dir,err=%s!\n",USB_ROOT,strerror(errno));
				return -1;
			}
		}
		else
		{
			printf("failed to open %s dir,err=%s\n!",USB_ROOT,strerror(errno));
			return -1;
		}
	}

	d = opendir("/dev/block");
	if(d != NULL) {
		while(de = readdir(d)) {
			printf("/dev/block/%s\n", de->d_name);
			if((strncmp(de->d_name, "sd", 2) == 0) &&(isdigit(de->d_name[strlen(de->d_name)-1])!=0)){
				memset(usbDevice, 0, sizeof(usbDevice));
				sprintf(usbDevice, "/dev/block/%s", de->d_name);
				printf("try to mount usb device %s by vfat", usbDevice);
				result = mount(usbDevice, USB_ROOT, "vfat",
						MS_NOATIME | MS_NODEV | MS_NODIRATIME, "shortname=mixed,utf8");
				if(result != 0) {
					printf("try to mount usb device %s by ntfs\n", usbDevice);
					result = mount(usbDevice, USB_ROOT, "ntfs",
							MS_NOATIME | MS_NODEV | MS_NODIRATIME, "");
				}

				if(result == 0) {
					strcpy(USB_DEVICE_PATH,usbDevice);
					closedir(d);
					return 0;
				}
			}
		}
		closedir(d);
	}

	return -2;
}

void ensure_sd_mounted()
{
    int i;
    for(i = 0; i < 3; i++) {
        if(0 == ensure_path_mounted(EX_SDCARD_ROOT)){
            bSDMounted = true;
            break;
        }else {
            printf("delay 1sec\n");
            sleep(1);
        }
    }
}

void ensure_usb_mounted()
{
	int i;
    for(i = 0; i < 3; i++) {
		if(0 == mount_usb_device()){
			bUsbMounted = true;
			break;
		}else {
			printf("delay 1sec\n");
			sleep(1);
		}
	}
}

static bool
get_args_from_sd(int *argc, char ***argv,bool *bmalloc)
{
	*bmalloc = false;
	ensure_sd_mounted();
	if (!bSDMounted)
	{
		printf("out get_args_from_sd:bSDMounted\n");
		return false;
	}
	char configFile[64];
	char arg[64];
	strcpy(configFile, EX_SDCARD_ROOT);
	strcat(configFile, "/sd_boot_config.config");
	VEC_SD_CONFIG vecItem;
	int i;
	if (!parse_config_file(configFile,vecItem))
	{
		printf("out get_args_from_sd:parse_config_file\n");
		return false;
	}

	*argv = (char **) malloc(sizeof(char *) * MAX_ARGS);
	*bmalloc = true;
    (*argv)[0] = strdup("recovery");
	(*argc) = 1;

	for (i=0;i<vecItem.size();i++)
	{
		if ((strcmp(vecItem[i].strKey.c_str(),"pcba_test")==0)||
		   (strcmp(vecItem[i].strKey.c_str(),"fw_update")==0)||
		   (strcmp(vecItem[i].strKey.c_str(),"demo_copy")==0)||
		   (strcmp(vecItem[i].strKey.c_str(),"volume_label")==0))
		{
			if (strcmp(vecItem[i].strValue.c_str(),"0")!=0)
			{
				sprintf(arg,"--%s=%s",vecItem[i].strKey.c_str(),vecItem[i].strValue.c_str());
				printf("%s\n",arg);
				(*argv)[*argc] = strdup(arg);
				(*argc)++;
			}
		}
	}
	printf("out get_args_from_sd\n");
	return true;

}

static bool
get_args_from_usb(int *argc, char ***argv,bool *bmalloc)
{
	printf("in get_args_from_usb\n");
	*bmalloc = false;
	ensure_usb_mounted();
	if (!bUsbMounted)
	{
		printf("out get_args_from_usb:bUsbMounted=false\n");
		return false;
	}
	
	char configFile[64];
	char arg[64];
	strcpy(configFile, USB_ROOT);
	strcat(configFile, "/sd_boot_config.config");
	VEC_SD_CONFIG vecItem;
	int i;
	if (!parse_config_file(configFile,vecItem))
	{
		printf("out get_args_from_usb:parse_config_file\n");
		return false;
	}

	*argv = (char **) malloc(sizeof(char *) * MAX_ARGS);
	*bmalloc = true;
    (*argv)[0] = strdup("recovery");
	(*argc) = 1;

	for (i=0;i<vecItem.size();i++)
	{
		if ((strcmp(vecItem[i].strKey.c_str(),"pcba_test")==0)||
		   (strcmp(vecItem[i].strKey.c_str(),"fw_update")==0)||
		   (strcmp(vecItem[i].strKey.c_str(),"demo_copy")==0)||
		   (strcmp(vecItem[i].strKey.c_str(),"volume_label")==0))
		{
			if (strcmp(vecItem[i].strValue.c_str(),"0")!=0)
			{
				sprintf(arg,"--%s=%s",vecItem[i].strKey.c_str(),vecItem[i].strValue.c_str());
				printf("%s\n",arg);
				(*argv)[*argc] = strdup(arg);
				(*argc)++;
			}
		}
	}
	printf("out get_args_from_usb\n");
	return true;
}
// command line args come from, in decreasing precedence:
//   - the actual command line
//   - the bootloader control block (one per line, after "recovery")
//   - the contents of COMMAND_FILE (one per line)
static void
get_args(int *argc, char ***argv) {
    struct bootloader_message boot;
    memset(&boot, 0, sizeof(boot));
    get_bootloader_message(&boot);  // this may fail, leaving a zeroed structure
    stage = strndup(boot.stage, sizeof(boot.stage));

    if (boot.command[0] != 0 && boot.command[0] != 255) {
        LOGI("Boot command: %.*s\n", (int)sizeof(boot.command), boot.command);
    }

    if (boot.status[0] != 0 && boot.status[0] != 255) {
        LOGI("Boot status: %.*s\n", (int)sizeof(boot.status), boot.status);
    }

    // --- if arguments weren't supplied, look in the bootloader control block
    if (*argc <= 1) {
        boot.recovery[sizeof(boot.recovery) - 1] = '\0';  // Ensure termination
        const char *arg = strtok(boot.recovery, "\n");
        if (arg != NULL && !strcmp(arg, "recovery")) {
            *argv = (char **) malloc(sizeof(char *) * MAX_ARGS);
            (*argv)[0] = strdup(arg);
            for (*argc = 1; *argc < MAX_ARGS; ++*argc) {
                if ((arg = strtok(NULL, "\n")) == NULL) break;
                (*argv)[*argc] = strdup(arg);
            }
            LOGI("Got arguments from boot message\n");
        } else if (boot.recovery[0] != 0 && boot.recovery[0] != 255) {
            LOGE("Bad boot message\n\"%.20s\"\n", boot.recovery);
        }
    }

    // --- if that doesn't work, try the command file
    if (*argc <= 1) {
        FILE *fp = fopen_path(COMMAND_FILE, "r");
        if (fp != NULL) {
            char *token;
            char *argv0 = (*argv)[0];
            *argv = (char **) malloc(sizeof(char *) * MAX_ARGS);
            (*argv)[0] = argv0;  // use the same program name

            char buf[MAX_ARG_LENGTH];
            for (*argc = 1; *argc < MAX_ARGS; ++*argc) {
                if (!fgets(buf, sizeof(buf), fp)) break;
                token = strtok(buf, "\r\n");
                if (token != NULL) {
                    (*argv)[*argc] = strdup(token);  // Strip newline.
                } else {
                    --*argc;
                }
            }

            check_and_fclose(fp, COMMAND_FILE);
            LOGI("Got arguments from %s\n", COMMAND_FILE);
        }
    }

    // --> write the arguments we have back into the bootloader control block
    // always boot into recovery after this (until finish_recovery() is called)
    strlcpy(boot.command, "boot-recovery", sizeof(boot.command));
    strlcpy(boot.recovery, "recovery\n", sizeof(boot.recovery));
    int i;
    for (i = 1; i < *argc; ++i) {
        strlcat(boot.recovery, (*argv)[i], sizeof(boot.recovery));
        strlcat(boot.recovery, "\n", sizeof(boot.recovery));
    }
    set_bootloader_message(&boot);
}

static void
set_sdcard_update_bootloader_message() {
    struct bootloader_message boot;
    memset(&boot, 0, sizeof(boot));
    strlcpy(boot.command, "boot-recovery", sizeof(boot.command));
    strlcpy(boot.recovery, "recovery\n", sizeof(boot.recovery));
    set_bootloader_message(&boot);
}

// Read from kernel log into buffer and write out to file.
static void save_kernel_log(const char* destination) {
    int n;
    char *buffer;
    int klog_buf_len;
    FILE *log;

    klog_buf_len = klogctl(KLOG_SIZE_BUFFER, 0, 0);
    if (klog_buf_len <= 0) {
        LOGE("Error getting klog size (%s), using default\n", strerror(errno));
        klog_buf_len = KLOG_DEFAULT_LEN;
    }

    buffer = (char *)malloc(klog_buf_len);
    if (!buffer) {
        LOGE("Can't alloc %d bytes for klog buffer\n", klog_buf_len);
        return;
    }

    n = klogctl(KLOG_READ_ALL, buffer, klog_buf_len);
    if (n < 0) {
        LOGE("Error in reading klog (%s)\n", strerror(errno));
        free(buffer);
        return;
    }

    log = fopen_path(destination, "w");
    if (log == NULL) {
        LOGE("Can't open %s\n", destination);
        free(buffer);
        return;
    }
    fwrite(buffer, n, 1, log);
    check_and_fclose(log, destination);
    free(buffer);
}

// How much of the temp log we have copied to the copy in cache.
static long tmplog_offset = 0;

static void copy_log_file(const char* source, const char* destination, bool append) {
    FILE* dest_fp = fopen_path(destination, append ? "a" : "w");
    if (dest_fp == nullptr) {
        LOGE("Can't open %s\n", destination);
    } else {
        FILE* source_fp = fopen(source, "r");
        if (source_fp != nullptr) {
            if (append) {
                fseek(source_fp, tmplog_offset, SEEK_SET);  // Since last write
            }
            char buf[4096];
            size_t bytes;
            while ((bytes = fread(buf, 1, sizeof(buf), source_fp)) != 0) {
                fwrite(buf, 1, bytes, dest_fp);
            }
            if (append) {
                tmplog_offset = ftell(source_fp);
            }
            check_and_fclose(source_fp, source);
        }
        check_and_fclose(dest_fp, destination);
    }
}

// Rename last_log -> last_log.1 -> last_log.2 -> ... -> last_log.$max.
// Similarly rename last_kmsg -> last_kmsg.1 -> ... -> last_kmsg.$max.
// Overwrite any existing last_log.$max and last_kmsg.$max.
static void rotate_logs(int max) {
    // Logs should only be rotated once.
    static bool rotated = false;
    if (rotated) {
        return;
    }
    rotated = true;
    ensure_path_mounted(LAST_LOG_FILE);
    ensure_path_mounted(LAST_KMSG_FILE);

    for (int i = max-1; i >= 0; --i) {

        char old_log[100], new_log[100], old_kmsg[100], new_kmsg[100];

        sprintf(old_log, (i == 0) ? "%s" : "%s.%d", LAST_LOG_FILE, i);
        sprintf(new_log, "%s.%d", LAST_LOG_FILE, i+1);
        rename(old_log, new_log);

        sprintf(old_kmsg, (i == 0) ? "%s" : "%s.%d", LAST_KMSG_FILE, i);
        sprintf(new_kmsg, "%s.%d", LAST_KMSG_FILE, i+1);
        rename(old_kmsg, new_kmsg);
        /* aosp code
        std::string old_log = android::base::StringPrintf((i == 0) ? "%s" : "%s.%d",
                LAST_LOG_FILE, i);
        std::string new_log = android::base::StringPrintf("%s.%d", LAST_LOG_FILE, i+1);
        // Ignore errors if old_log doesn't exist.
        rename(old_log.c_str(), new_log.c_str());

        std::string old_kmsg = android::base::StringPrintf((i == 0) ? "%s" : "%s.%d",
                LAST_KMSG_FILE, i);
        std::string new_kmsg = android::base::StringPrintf("%s.%d", LAST_KMSG_FILE, i+1);
        rename(old_kmsg.c_str(), new_kmsg.c_str());*/
    }
}

static void copy_logs() {
    // We only rotate and record the log of the current session if there are
    // actual attempts to modify the flash, such as wipes, installs from BCB
    // or menu selections. This is to avoid unnecessary rotation (and
    // possible deletion) of log files, if it does not do anything loggable.
    if (!modified_flash) {
        return;
    }

    rotate_logs(KEEP_LOG_COUNT);

    // Copy logs to cache so the system can find out what happened.
    copy_log_file(TEMPORARY_LOG_FILE, LOG_FILE, true);
    copy_log_file(TEMPORARY_LOG_FILE, LAST_LOG_FILE, false);
    copy_log_file(TEMPORARY_INSTALL_FILE, LAST_INSTALL_FILE, false);
    save_kernel_log(LAST_KMSG_FILE);
    chmod(LOG_FILE, 0600);
    chown(LOG_FILE, 1000, 1000);   // system user
    chmod(LAST_KMSG_FILE, 0600);
    chown(LAST_KMSG_FILE, 1000, 1000);   // system user
    chmod(LAST_LOG_FILE, 0640);
    chmod(LAST_INSTALL_FILE, 0644);
    sync();
}

// clear the recovery command and prepare to boot a (hopefully working) system,
// copy our log file to cache as well (for the system to read), and
// record any intent we were asked to communicate back to the system.
// this function is idempotent: call it as many times as you like.
static void
finish_recovery(const char *send_intent) {
    // By this point, we're ready to return to the main system...
    if (send_intent != NULL) {
        FILE *fp = fopen_path(INTENT_FILE, "w");
        if (fp == NULL) {
            LOGE("Can't open %s\n", INTENT_FILE);
        } else {
            fputs(send_intent, fp);
            check_and_fclose(fp, INTENT_FILE);
        }
    }

    // Save the locale to cache, so if recovery is next started up
    // without a --locale argument (eg, directly from the bootloader)
    // it will use the last-known locale.
    if (locale != NULL) {
        LOGI("Saving locale \"%s\"\n", locale);
        FILE* fp = fopen_path(LOCALE_FILE, "w");
        fwrite(locale, 1, strlen(locale), fp);
        fflush(fp);
        fsync(fileno(fp));
        check_and_fclose(fp, LOCALE_FILE);
    }
    #ifdef LogToCache
    copy_logs();
    #endif

    // Reset to normal system boot so recovery won't cycle indefinitely.
    if( bNeedClearMisc ) {
    	struct bootloader_message boot;
    	memset(&boot, 0, sizeof(boot));
    	set_bootloader_message(&boot);
    }
	
 	if (bAutoUpdateComplete==true) {
		FILE *fp = fopen_path(FLAG_FILE, "w");
		if (fp == NULL) {
			LOGE("Can't open %s\n", FLAG_FILE);
		}
		char strflag[160]="success$path=";
		strcat(strflag,updatepath);
		if (fwrite(strflag, 1, sizeof(strflag), fp) != sizeof(strflag)) {
			LOGE("write %s failed! \n", FLAG_FILE);
		}
		fclose(fp);
		bAutoUpdateComplete=false;
     }

    // Remove the command file, so recovery won't repeat indefinitely.
    if (ensure_path_mounted(COMMAND_FILE) != 0 ||
        (unlink(COMMAND_FILE) && errno != ENOENT)) {
        LOGW("Can't unlink %s\n", COMMAND_FILE);
    }

    ensure_path_unmounted(CACHE_ROOT);
    sync();  // For good measure.
}

typedef struct _saved_log_file {
    char* name;
    struct stat st;
    unsigned char* data;
    struct _saved_log_file* next;
} saved_log_file;

static bool erase_volume(const char* volume) {
    bool is_cache = (strcmp(volume, CACHE_ROOT) == 0);

    ui->SetBackground(RecoveryUI::ERASING);
    ui->SetProgressType(RecoveryUI::INDETERMINATE);

    saved_log_file* head = NULL;

    if (is_cache) {
        // If we're reformatting /cache, we load any past logs
        // (i.e. "/cache/recovery/last_*") and the current log
        // ("/cache/recovery/log") into memory, so we can restore them after
        // the reformat.

        ensure_path_mounted(volume);

        DIR* d;
        struct dirent* de;
        d = opendir(CACHE_LOG_DIR);
        if (d) {
            char path[PATH_MAX];
            strcpy(path, CACHE_LOG_DIR);
            strcat(path, "/");
            int path_len = strlen(path);
            while ((de = readdir(d)) != NULL) {
                if (strncmp(de->d_name, "last_", 5) == 0 || strcmp(de->d_name, "log") == 0 || strncmp(de->d_name, "Recovery_", 9) == 0) {
                    saved_log_file* p = (saved_log_file*) malloc(sizeof(saved_log_file));
                    strcpy(path+path_len, de->d_name);
                    p->name = strdup(path);
                    if (stat(path, &(p->st)) == 0) {
                        // truncate files to 512kb
                        if (p->st.st_size > (1 << 19)) {
                            p->st.st_size = 1 << 19;
                        }
                        p->data = (unsigned char*) malloc(p->st.st_size);
                        FILE* f = fopen(path, "rb");
                        fread(p->data, 1, p->st.st_size, f);
                        fclose(f);
                        p->next = head;
                        head = p;
                    } else {
                        free(p);
                    }
                }
            }
            closedir(d);
        } else {
            if (errno != ENOENT) {
                printf("opendir failed: %s\n", strerror(errno));
            }
        }
    }

    ui->Print("Formatting %s...\n", volume);

    ensure_path_unmounted(volume);
    int result = format_volume(volume);

    if (is_cache) {
        while (head) {
            FILE* f = fopen_path(head->name, "wb");
            if (f) {
                fwrite(head->data, 1, head->st.st_size, f);
                fclose(f);
                chmod(head->name, head->st.st_mode);
                chown(head->name, head->st.st_uid, head->st.st_gid);
            }
            free(head->name);
            free(head->data);
            saved_log_file* temp = head->next;
            free(head);
            head = temp;
        }

        // Any part of the log we'd copied to cache is now gone.
        // Reset the pointer so we copy from the beginning of the temp
        // log.
        tmplog_offset = 0;
        #ifdef LogToCache
        copy_logs();
        #endif
    }

    return (result == 0);
}

static int
get_menu_selection(const char* const * headers, const char* const * items,
                   int menu_only, int initial_selection, Device* device) {
    // throw away keys pressed previously, so user doesn't
    // accidentally trigger menu items.
    ui->FlushKeys();

    ui->StartMenu(headers, items, initial_selection);
    int selected = initial_selection;
    int chosen_item = -1;

    while (chosen_item < 0) {
        int key = ui->WaitKey();
        int visible = ui->IsTextVisible();

        if (key == -1) {   // ui_wait_key() timed out
            if (ui->WasTextEverVisible()) {
                continue;
            } else {
                LOGI("timed out waiting for key input; rebooting.\n");
                ui->EndMenu();
                return 0; // XXX fixme
            }
        }

        int action = device->HandleMenuKey(key, visible);

        if (action < 0) {
            switch (action) {
                case Device::kHighlightUp:
                    selected = ui->SelectMenu(--selected);
                    break;
                case Device::kHighlightDown:
                    selected = ui->SelectMenu(++selected);
                    break;
                case Device::kInvokeItem:
                    chosen_item = selected;
                    break;
                case Device::kNoAction:
                    break;
            }
        } else if (!menu_only) {
            chosen_item = action;
        }
    }

    ui->EndMenu();
    return chosen_item;
}

static int compare_string(const void* a, const void* b) {
    return strcmp(*(const char**)a, *(const char**)b);
}

// Returns a malloc'd path, or NULL.
static char* browse_directory(const char* path, Device* device) {
    ensure_path_mounted(path);

    DIR* d = opendir(path);
    if (d == NULL) {
        LOGE("error opening %s: %s\n", path, strerror(errno));
        return NULL;
    }

    int d_size = 0;
    int d_alloc = 10;
    char** dirs = (char**)malloc(d_alloc * sizeof(char*));
    int z_size = 1;
    int z_alloc = 10;
    char** zips = (char**)malloc(z_alloc * sizeof(char*));
    zips[0] = strdup("../");

    struct dirent* de;
    while ((de = readdir(d)) != NULL) {
        int name_len = strlen(de->d_name);

        if (de->d_type == DT_DIR) {
            // skip "." and ".." entries
            if (name_len == 1 && de->d_name[0] == '.') continue;
            if (name_len == 2 && de->d_name[0] == '.' &&
                de->d_name[1] == '.') continue;

            if (d_size >= d_alloc) {
                d_alloc *= 2;
                dirs = (char**)realloc(dirs, d_alloc * sizeof(char*));
            }
            dirs[d_size] = (char*)malloc(name_len + 2);
            strcpy(dirs[d_size], de->d_name);
            dirs[d_size][name_len] = '/';
            dirs[d_size][name_len+1] = '\0';
            ++d_size;
        } else if (de->d_type == DT_REG &&
                   name_len >= 4 &&
                   strncasecmp(de->d_name + (name_len-4), ".zip", 4) == 0) {
            if (z_size >= z_alloc) {
                z_alloc *= 2;
                zips = (char**)realloc(zips, z_alloc * sizeof(char*));
            }
            zips[z_size++] = strdup(de->d_name);
        }
    }
    closedir(d);

    qsort(dirs, d_size, sizeof(char*), compare_string);
    qsort(zips, z_size, sizeof(char*), compare_string);

    // append dirs to the zips list
    if (d_size + z_size + 1 > z_alloc) {
        z_alloc = d_size + z_size + 1;
        zips = (char**)realloc(zips, z_alloc * sizeof(char*));
    }
    memcpy(zips + z_size, dirs, d_size * sizeof(char*));
    free(dirs);
    z_size += d_size;
    zips[z_size] = NULL;

    const char* headers[] = { "Choose a package to install:", path, NULL };

    char* result;
    int chosen_item = 0;
    while (true) {
        chosen_item = get_menu_selection(headers, zips, 1, chosen_item, device);

        char* item = zips[chosen_item];
        int item_len = strlen(item);
        if (chosen_item == 0) {          // item 0 is always "../"
            // go up but continue browsing (if the caller is update_directory)
            result = NULL;
            break;
        }

        char new_path[PATH_MAX];
        strlcpy(new_path, path, PATH_MAX);
        strlcat(new_path, "/", PATH_MAX);
        strlcat(new_path, item, PATH_MAX);

        if (item[item_len-1] == '/') {
            // recurse down into a subdirectory
            new_path[strlen(new_path)-1] = '\0';  // truncate the trailing '/'
            result = browse_directory(new_path, device);
            if (result) break;
        } else {
            // selected a zip file: return the malloc'd path to the caller.
            result = strdup(new_path);
            break;
        }
    }

    for (int i = 0; i < z_size; ++i) free(zips[i]);
    free(zips);

    return result;
}

static bool yes_no(Device* device, const char* question1, const char* question2) {
    const char* headers[] = { question1, question2, NULL };
    const char* items[] = { " No", " Yes", NULL };

    int chosen_item = get_menu_selection(headers, items, 1, 0, device);
    return (chosen_item == 1);
}

// Return true on success.
static bool wipe_data(int should_confirm, Device* device) {
    if (should_confirm && !yes_no(device, "Wipe all user data?", "  THIS CAN NOT BE UNDONE!")) {
        return false;
    }

    modified_flash = true;

    ui->Print("\n-- Wiping data...\n");
    bool success =
        device->PreWipeData() &&
        erase_volume("/data") &&
        erase_volume("/cache") &&
        device->PostWipeData();
    ui->Print("Data wipe %s.\n", success ? "complete" : "failed");
    return success;
}

// Return true on success.
static bool wipe_cache(bool should_confirm, Device* device) {
    if (should_confirm && !yes_no(device, "Wipe cache?", "  THIS CAN NOT BE UNDONE!")) {
        return false;
    }

    modified_flash = true;

    ui->Print("\n-- Wiping cache...\n");
    bool success = erase_volume("/cache");
    ui->Print("Cache wipe %s.\n", success ? "complete" : "failed");
    return success;
}

static void choose_recovery_file(Device* device) {
    // "Back" + KEEP_LOG_COUNT * 2 + terminating nullptr entry
    char* entries[1 + KEEP_LOG_COUNT * 2 + 1];
    memset(entries, 0, sizeof(entries));

    unsigned int n = 0;

    // Add LAST_LOG_FILE + LAST_LOG_FILE.x
    // Add LAST_KMSG_FILE + LAST_KMSG_FILE.x
    for (int i = 0; i < KEEP_LOG_COUNT; i++) {
        char* log_file;
        if (asprintf(&log_file, (i == 0) ? "%s" : "%s.%d", LAST_LOG_FILE, i) == -1) {
            // memory allocation failure - return early. Should never happen.
            return;
        }
        if ((ensure_path_mounted(log_file) != 0) || (access(log_file, R_OK) == -1)) {
            free(log_file);
        } else {
            entries[n++] = log_file;
        }

        char* kmsg_file;
        if (asprintf(&kmsg_file, (i == 0) ? "%s" : "%s.%d", LAST_KMSG_FILE, i) == -1) {
            // memory allocation failure - return early. Should never happen.
            return;
        }
        if ((ensure_path_mounted(kmsg_file) != 0) || (access(kmsg_file, R_OK) == -1)) {
            free(kmsg_file);
        } else {
            entries[n++] = kmsg_file;
        }
    }

    entries[n++] = strdup("Back");

    const char* headers[] = { "Select file to view", nullptr };

    while (true) {
        int chosen_item = get_menu_selection(headers, entries, 1, 0, device);
        if (strcmp(entries[chosen_item], "Back") == 0) break;

        // TODO: do we need to redirect? ShowFile could just avoid writing to stdio.
        redirect_stdio("/dev/null");
        ui->ShowFile(entries[chosen_item]);
        redirect_stdio(TEMPORARY_LOG_FILE);
    }

    for (size_t i = 0; i < (sizeof(entries) / sizeof(*entries)); i++) {
        free(entries[i]);
    }
}

static int apply_from_sdcard(Device* device, bool* wipe_cache) {
    modified_flash = true;

    if (ensure_path_mounted(SDCARD_ROOT) != 0) {
        ui->Print("\n-- Couldn't mount %s.\n", SDCARD_ROOT);
        return INSTALL_ERROR;
    }

    char* path = browse_directory(SDCARD_ROOT, device);
    if (path == NULL) {
        ui->Print("\n-- No package file selected.\n");
        return INSTALL_ERROR;
    }

    ui->Print("\n-- Install %s ...\n", path);
    set_sdcard_update_bootloader_message();
    void* token = start_sdcard_fuse(path);

    int status = install_package(FUSE_SIDELOAD_HOST_PATHNAME, wipe_cache,
                                 TEMPORARY_INSTALL_FILE, false);

    finish_sdcard_fuse(token);
    ensure_path_unmounted(SDCARD_ROOT);
    return status;
}

// Return REBOOT, SHUTDOWN, or REBOOT_BOOTLOADER.  Returning NO_ACTION
// means to take the default, which is to reboot or shutdown depending
// on if the --shutdown_after flag was passed to recovery.
static Device::BuiltinAction
prompt_and_wait(Device* device, int status) {
    for (;;) {
        finish_recovery(NULL);
        switch (status) {
            case INSTALL_SUCCESS:
            case INSTALL_NONE:
                ui->SetBackground(RecoveryUI::NO_COMMAND);
                break;

            case INSTALL_ERROR:
            case INSTALL_CORRUPT:
                ui->SetBackground(RecoveryUI::ERROR);
                break;
        }
        ui->SetProgressType(RecoveryUI::EMPTY);

        int chosen_item = get_menu_selection(nullptr, device->GetMenuItems(), 0, 0, device);

        // device-specific code may take some action here.  It may
        // return one of the core actions handled in the switch
        // statement below.
        Device::BuiltinAction chosen_action = device->InvokeMenuItem(chosen_item);

        bool should_wipe_cache = false;
        switch (chosen_action) {
            case Device::NO_ACTION:
                break;

            case Device::REBOOT:
            case Device::SHUTDOWN:
            case Device::REBOOT_BOOTLOADER:
                return chosen_action;

            case Device::WIPE_DATA:
                wipe_data(ui->IsTextVisible(), device);
                if (!ui->IsTextVisible()) return Device::NO_ACTION;
                break;

            case Device::WIPE_CACHE:
                wipe_cache(ui->IsTextVisible(), device);
                if (!ui->IsTextVisible()) return Device::NO_ACTION;
                break;

            case Device::APPLY_ADB_SIDELOAD:
            case Device::APPLY_SDCARD:
                {
                    bool adb = (chosen_action == Device::APPLY_ADB_SIDELOAD);
                    if (adb) {
                        status = apply_from_adb(ui, &should_wipe_cache, TEMPORARY_INSTALL_FILE);
                    } else {
                        status = apply_from_sdcard(device, &should_wipe_cache);
                    }

                    if (status == INSTALL_SUCCESS && should_wipe_cache) {
                        if (!wipe_cache(false, device)) {
                            status = INSTALL_ERROR;
                        }
                    }

                    if (status != INSTALL_SUCCESS) {
                        ui->SetBackground(RecoveryUI::ERROR);
                        ui->Print("Installation aborted.\n");
                        #ifdef LogToCache
                        copy_logs();
                        #endif
                    } else if (!ui->IsTextVisible()) {
                        return Device::NO_ACTION;  // reboot if logs aren't visible
                    } else {
                        ui->Print("\nInstall from %s complete.\n", adb ? "ADB" : "SD card");
                    }
                }
                break;

            case Device::VIEW_RECOVERY_LOGS:
                choose_recovery_file(device);
                break;

            case Device::MOUNT_SYSTEM:
                if (ensure_path_mounted("/system") != -1) {
                    ui->Print("Mounted /system.\n");
                }
                break;
        }
    }
}

static void
print_property(const char *key, const char *name, void *cookie) {
    printf("%s=%s\n", key, name);
}

void SetSdcardRootPath(void)
{
     property_get("InternalSD_ROOT", IN_SDCARD_ROOT, "");
	 LOGI("InternalSD_ROOT: %s\n", IN_SDCARD_ROOT);
	 property_get("ExternalSD_ROOT", EX_SDCARD_ROOT, "");
	 LOGI("ExternalSD_ROOT: %s\n", EX_SDCARD_ROOT);

	 return;
}

static void
load_locale_from_cache() {
    FILE* fp = fopen_path(LOCALE_FILE, "r");
    char buffer[80];
    if (fp != NULL) {
        fgets(buffer, sizeof(buffer), fp);
        int j = 0;
        unsigned int i;
        for (i = 0; i < sizeof(buffer) && buffer[i]; ++i) {
            if (!isspace(buffer[i])) {
                buffer[j++] = buffer[i];
            }
        }
        buffer[j] = 0;
        locale = strdup(buffer);
        check_and_fclose(fp, LOCALE_FILE);
    }
}

static RecoveryUI* gCurrentUI = NULL;

void
ui_print(const char* format, ...) {
    char buffer[256];

    va_list ap;
    va_start(ap, format);
    vsnprintf(buffer, sizeof(buffer), format, ap);
    va_end(ap);

    if (gCurrentUI != NULL) {
        gCurrentUI->Print("%s", buffer);
    } else {
        fputs(buffer, stdout);
    }
}

void SureMetadataMount() {
    if (ensure_path_mounted("/metadata") != 0) {
        printf("mount metadata fail,so formate...\n");
        tmplog_offset = 0;
        format_volume("/metadata");
        ensure_path_mounted("/metadata");
    }
}

void SureCacheMount() {
    if(ensure_path_mounted("/cache") != 0) {
        printf("mount cache fail,so formate...\n");
        tmplog_offset = 0;
        format_volume("/cache");
        ensure_path_mounted("/cache");
    }
}

ssize_t mygetline(char **lineptr, size_t *n, FILE *stream) {
	if(*n <= 0) {
		*lineptr = (char*)malloc(128);
		memset(*lineptr, 0, 128);
		*n = 128;
	}

	char c;
	char *pline = *lineptr;
	size_t count = 0;
	while((fread(&c, 1, 1, stream)) == 1) {
		if(c == '\n') {
			*pline = '\0';
			return count;
		}else if(c == '\r') {
			fread(&c, 1, 1, stream);
			if(c == '\n'){
				*pline = '\0';
				return count;
			}
		}else {
			if(count >= *n -1) {
				*lineptr = (char*)realloc(*lineptr, *n + 128);
				*n = *n + 128;
				pline = *lineptr + count;
			}

			*pline = c;
			count++;
			pline++;
		}
	}

	*pline = '\0';
	if(count == 0) {
		return -1;
	}

	return count;
}
char* readConfig(FILE *pCfgFile, char const *keyName) {
	char *keyValue = NULL;
	char *line = NULL;
	size_t len = 0;
	ssize_t read = 0;

	fseek(pCfgFile, 0, SEEK_SET);
	while((read = mygetline(&line, &len, pCfgFile)) != -1) {

		char *pstr = line;
		if(*pstr == '#' || *pstr == '\0') {
			continue;
		}
		printf("get line %s\n", pstr);
		if(strstr(pstr, keyName) != NULL) {
			char *pValue = strchr(pstr, '=');
			if(pValue != NULL) {
				keyValue = (char*)malloc(strlen(pValue) + 1);
				while(*(++pValue) == ' ');
				strcpy(keyValue, pValue);
				printf("find property %s value %s\n", keyName, keyValue);
				break;
			}
		}
	}

	printf("read config end\n");
	free(line);
	return keyValue;
}
void parseSDBootConfig() {
	char configPath[64];
	strcpy(configPath, EX_SDCARD_ROOT);
	strcat(configPath, "/sd_boot_config.config");

	printf("sd boot config file is %s", configPath);
	FILE* configFile = fopen(configPath, "rb");

	if(!configFile) {
		printf("no find config file!\n");
		return;
	}

	char *pValue;
	if((pValue = readConfig(configFile, SdBootConfigs[pcba_test].name)) != NULL) {
		printf("set pcba_test value is %s\n", pValue);
		SdBootConfigs[pcba_test].value = pValue;
	}

	if((pValue = readConfig(configFile, SdBootConfigs[fw_update].name)) != NULL) {
		printf("set fw_update value is %s\n", pValue);
		SdBootConfigs[fw_update].value = pValue;
	}

	if((pValue = readConfig(configFile, SdBootConfigs[display_led].name)) != NULL) {
		printf("set display_led value is %s\n", pValue);
		SdBootConfigs[display_led].value = pValue;
	}

	if((pValue = readConfig(configFile, SdBootConfigs[display_lcd].name)) != NULL) {
		printf("set display_lcd value is %s\n", pValue);
		SdBootConfigs[display_lcd].value = pValue;
	}

	if((pValue = readConfig(configFile, SdBootConfigs[demo_copy].name)) != NULL) {
		printf("set demo_copy value is %s\n", pValue);
		SdBootConfigs[demo_copy].value = pValue;
	}

	if((pValue = readConfig(configFile, SdBootConfigs[volume_label].name)) != NULL) {
		printf("set volume_label value is %s\n", pValue);
		SdBootConfigs[volume_label].value = pValue;
		strcpy(gVolume_label, SdBootConfigs[volume_label].value);
	}
}

int do_sd_mode_update(const char *pFile)
{
    int status=INSTALL_SUCCESS;
    bool bRet,bUpdateIDBlock=true;
    char *pFwPath = (char *)malloc(100);
    strcpy(pFwPath, EX_SDCARD_ROOT);
    if (strcmp(pFile,"1")==0)
    {
        strcat(pFwPath, "/sdupdate.img");
    }
    else if (strcmp(pFile,"2")==0)
    {
        strcat(pFwPath, "/sdupdate.img");
        bUpdateIDBlock = false;
    }
    else
    {
        strcat(pFwPath, pFile);
    }
    //format user
//  erase_volume(IN_SDCARD_ROOT);
//  //format userdata
//    if (clone_data_if_exist()) {
//        if (erase_volume("/data")) return INSTALL_ERROR;
//    }
    ui->SetBackground(RecoveryUI::INSTALLING_UPDATE);
    ui->SetProgressType(RecoveryUI::DETERMINATE);
    printf("start sd upgrade...\n");

#ifdef USE_BOARD_ID
    ensure_path_mounted("/cust");
    ensure_path_mounted("/system");
    restore();
#endif
    if (bUpdateIDBlock)
        bRet= do_rk_firmware_upgrade(pFwPath,(void *)handle_upgrade_callback,(void *)handle_upgrade_progress_callback);
    else
        bRet = do_rk_partition_upgrade(pFwPath,(void *)handle_upgrade_callback,(void *)handle_upgrade_progress_callback);
    ui->SetProgressType(RecoveryUI::EMPTY);
    if (!bRet)
    {
        status = INSTALL_ERROR;
        ui->Print("SD upgrade failed!\n");
    }
    else
    {
#ifdef USE_BOARD_ID
        ensure_path_mounted("/cust");
        ensure_path_mounted("/system");
        custom();
#endif
        status = INSTALL_SUCCESS;
//      if(bIfUpdateLoader)
//      {
//          bNeedClearMisc = false;
//      }
        bAutoUpdateComplete=true;
        printf("SD upgrade ok.\n");
    }
        
    return status;
}

int do_usb_mode_update(char *pFile)
{
    printf("in do_usb_mode_update\n");
    int status=INSTALL_SUCCESS;
    bool bRet,bUpdateIDBlock=true;
    char szDev[100];
    char *pFwPath = (char *)malloc(100);
    strcpy(pFwPath, USB_ROOT);
    if (strcmp(pFile,"1")==0)
    {
        strcat(pFwPath, "/sdupdate.img");
    }
    else if (strcmp(pFile,"2")==0)
    {
        strcat(pFwPath, "/sdupdate.img");
        bUpdateIDBlock = false;
    }
    else
    {
        strcat(pFwPath, pFile);
    }
    //format user
//  erase_volume(IN_SDCARD_ROOT);
//  //format userdata
//    if (clone_data_if_exist()) {
//        if (erase_volume("/data")) return INSTALL_ERROR;
//    }
    strcpy(szDev,USB_DEVICE_PATH);
    if (strlen(szDev)>0)
        szDev[strlen(szDev)-1]=0;
    
    ui->SetBackground(RecoveryUI::INSTALLING_UPDATE);
    ui->SetProgressType(RecoveryUI::DETERMINATE);
    ui->Print("UDisk upgrade...\n");
    if (access(pFwPath,F_OK)!=0)
    {
        ui->SetProgressType(RecoveryUI::EMPTY);
        ui->Print("Firmware is not existed,file=%s!\n",pFwPath);
        return INSTALL_ERROR;
    }

    if (bUpdateIDBlock)
        bRet= do_rk_firmware_upgrade(pFwPath,(void *)handle_upgrade_callback,(void *)handle_upgrade_progress_callback,szDev);
    else
        bRet = do_rk_partition_upgrade(pFwPath,(void *)handle_upgrade_callback,(void *)handle_upgrade_progress_callback,2,szDev);
    ui->SetProgressType(RecoveryUI::EMPTY);
    if (!bRet)
    {
        status = INSTALL_ERROR;
        printf("USB upgrade failed!\n");
    }
    else
    {
        status = INSTALL_SUCCESS;
//      if(bIfUpdateLoader)
//      {
//          bNeedClearMisc = false;
//      }
        bAutoUpdateComplete=true;
        printf("USB upgrade ok.\n");
    }
        
    return status;
}


void checkSDRemoved() {
	Volume* v = volume_for_path("/mnt/external_sd");
	char *temp;
	char *sec_dev = v->fs_options;
	if(sec_dev != NULL) {
		temp = strchr(sec_dev, ',');
		if(temp) {
			temp[0] = '\0';
		}
	}

	while(1) {
		int value2 = -1;
		int value = access(v->blk_device, 0);
		if(sec_dev) {
			value2 = access(sec_dev, 0);
		}
		if(value == -1 && value2 == -1) {
			printf("remove sdcard\n");
			break;
		}else {
			sleep(1);
		}
	}
}
void checkUSBRemoved() {
	int ret;

	while(1) {
		ret = access(USB_DEVICE_PATH, F_OK);

		if(ret==-1) {
			printf("remove USB\n");
			break;
		}else {
			sleep(1);
		}
	}
}
int do_sd_demo_copy(char *demoPath)
{
	printf("in do_sd_demo_copy\n");
	int status = INSTALL_SUCCESS;

	if(ensure_path_mounted(IN_SDCARD_ROOT)) {
		printf("mount user partition error!\n");
		return INSTALL_ERROR;
	}
	ui->SetBackground(RecoveryUI::INSTALLING_UPDATE);
	ui->SetProgressType(RecoveryUI::INDETERMINATE);
	ui->Print("Copying demo...\n");
	//copy demo files
	char *srcPath = (char *)malloc(strlen(EX_SDCARD_ROOT) + 64);
	strcpy(srcPath, EX_SDCARD_ROOT);
	if (strcmp(demoPath,"1")==0)
		strcat(srcPath, "/Demo");
	else
		strcat(srcPath, demoPath);
	if (access(srcPath,F_OK)!=0)
	{
		ui->SetProgressType(RecoveryUI::EMPTY);
		ui->Print("Demo is not existed,demo=%s!\n",srcPath);
		free(srcPath);
		return INSTALL_ERROR;
	}

	char *args[6];
	args[0] = strdup("/sbin/busybox");
	args[1] = strdup("cp");
	args[2] = strdup("-R");
	args[3] = strdup(srcPath);
	args[4] = strdup(IN_SDCARD_ROOT);
	args[5] = NULL;

	pid_t child = fork();
	if (child == 0) {
		printf("run busybox copy demo files...\n");
		execv(args[0], &args[1]);
		fprintf(stderr, "run_program: execv failed: %s\n", strerror(errno));
		_exit(1);
	}
	int child_status;
	waitpid(child, &child_status, 0);
	if (WIFEXITED(child_status)) {
		if (WEXITSTATUS(child_status) != 0) {
			status = INSTALL_ERROR;
			fprintf(stderr, "run_program: child exited with status %d\n",
					WEXITSTATUS(child_status));
		}
	} else if (WIFSIGNALED(child_status)) {
		status = INSTALL_ERROR;
		fprintf(stderr, "run_program: child terminated by signal %d\n",
				WTERMSIG(child_status));
	}

	free(srcPath);
	ui->SetProgressType(RecoveryUI::EMPTY);
	return status;
}
int do_usb_demo_copy(char *demoPath)
{
	printf("in do_usb_demo_copy\n");
	int status = INSTALL_SUCCESS;

	if(ensure_path_mounted(IN_SDCARD_ROOT)) {
		printf("mount user partition error!\n");
		return INSTALL_ERROR;
	}
	ui->SetBackground(RecoveryUI::INSTALLING_UPDATE);
	ui->SetProgressType(RecoveryUI::INDETERMINATE);
	ui->Print("Copying demo...\n");
	//copy demo files
	char *srcPath = (char *)malloc(strlen(USB_ROOT) + 64);
	strcpy(srcPath, USB_ROOT);
	if (strcmp(demoPath,"1")==0)
		strcat(srcPath, "/Demo");
	else
		strcat(srcPath, demoPath);

	if (access(srcPath,F_OK)!=0)
	{
		ui->SetProgressType(RecoveryUI::EMPTY);
		ui->Print("Demo is not existed,demo=%s!\n",srcPath);
		free(srcPath);
		return INSTALL_ERROR;
	}

	char *args[6];
	args[0] = strdup("/sbin/busybox");
	args[1] = strdup("cp");
	args[2] = strdup("-R");
	args[3] = strdup(srcPath);
	args[4] = strdup(IN_SDCARD_ROOT);
	args[5] = NULL;

	pid_t child = fork();
	if (child == 0) {
		printf("run busybox copy demo files...\n");
		execv(args[0], &args[1]);
		fprintf(stderr, "run_program: execv failed: %s\n", strerror(errno));
		_exit(1);
	}
	int child_status;
	waitpid(child, &child_status, 0);
	if (WIFEXITED(child_status)) {
		if (WEXITSTATUS(child_status) != 0) {
			status = INSTALL_ERROR;
			fprintf(stderr, "run_program: child exited with status %d\n",
					WEXITSTATUS(child_status));
		}
	} else if (WIFSIGNALED(child_status)) {
		status = INSTALL_ERROR;
		fprintf(stderr, "run_program: child terminated by signal %d\n",
				WTERMSIG(child_status));
	}

	free(srcPath);
	ui->SetProgressType(RecoveryUI::EMPTY);
	return status;
}
int sdtool_main(char *factory_mode, Device* device) {
	ensure_path_mounted(IN_SDCARD_ROOT);
	parseSDBootConfig();
	int status = INSTALL_SUCCESS;
	bool pcbaTestPass = true;

	if(!strcmp(SdBootConfigs[pcba_test].value, "1") || bFactoryMode == true) {
		//pcba test
		printf("enter pcba test!\n");

		const char *args[2];
		args[0] = "/sbin/pcba_core";
		args[1] = NULL;

		pid_t child = fork();
		if (child == 0) {
			execv(args[0], (char* const*)args);
			fprintf(stderr, "run_program: execv failed: %s\n", strerror(errno));
			status = INSTALL_ERROR;
			pcbaTestPass = false;
		}
		int child_status;
		waitpid(child, &child_status, 0);
		if (WIFEXITED(child_status)) {
			if (WEXITSTATUS(child_status) != 0) {
				printf("pcba test error coder is %d \n", WEXITSTATUS(child_status));
				status = INSTALL_ERROR;
				pcbaTestPass = false;
			}
		} else if (WIFSIGNALED(child_status)) {
			printf("run_program: child terminated by signal %d\n", WTERMSIG(child_status));
			status = INSTALL_ERROR;
			pcbaTestPass = false;
		}
	}

	ui->Print("sdcard boot tools system v1.34 \n\n");
	//ui_set_background(BACKGROUND_ICON_INSTALLING);
/*
	if(!pcbaTestPass) {
		ui->Print("pcba test error!!!");
		goto finish;
	}
*/
	//format user partition
	property_get("UserVolumeLabel", gVolume_label, SdBootConfigs[volume_label].value);
	erase_volume(IN_SDCARD_ROOT);
	//format userdata

    if (erase_volume("/data")) status = INSTALL_ERROR;

	if(!strcmp(SdBootConfigs[fw_update].value, "1")) {
		//fw update
		ui->Print("fw update!!!");
		char *updateImagePath = (char*)malloc(100);
		strcpy(updateImagePath, EX_SDCARD_ROOT);
		strcat(updateImagePath, "/sdupdate.img");

		status = install_rkimage(updateImagePath);
		if(status != INSTALL_SUCCESS) {
			goto finish;
		}
	}

	if(!strcmp(SdBootConfigs[demo_copy].value, "1")) {

		if(ensure_path_mounted(IN_SDCARD_ROOT)) {
			printf("mount user partition error!\n");
			goto finish;
		}

		//copy demo files
		char *demoPath = (char*)malloc(strlen(EX_SDCARD_ROOT) + 64);

		strcpy(demoPath, EX_SDCARD_ROOT);
		strcat(demoPath, "/Demo");

		const char *args[6];
		args[0] = "/sbin/busybox";
		args[1] = "cp";
		args[2] = "-R";
		args[3] = demoPath;
		args[4] = IN_SDCARD_ROOT;
		args[5] = NULL;

		pid_t child = fork();
		if (child == 0) {
			printf("run busybox copy demo files...\n");
			execv(args[0], (char* const*)&args[1]);
			fprintf(stderr, "run_program: execv failed: %s\n", strerror(errno));
			_exit(1);
		}
		int child_status;
		waitpid(child, &child_status, 0);
		if (WIFEXITED(child_status)) {
			if (WEXITSTATUS(child_status) != 0) {
				fprintf(stderr, "run_program: child exited with status %d\n",
						WEXITSTATUS(child_status));
			}
		} else if (WIFSIGNALED(child_status)) {
			fprintf(stderr, "run_program: child terminated by signal %d\n",
					WTERMSIG(child_status));
		}

		free(demoPath);
	}

finish:
	if (status != INSTALL_SUCCESS) ui->SetBackground(RecoveryUI::ERROR);
	if (status != INSTALL_SUCCESS) {
		bNeedClearMisc = false;
		ui->ShowText(true);
		prompt_and_wait(device, status);
	}else {
		if((factory_mode != NULL && !strcmp(factory_mode, "small")) || bIfUpdateLoader == true) {
			printf("small fw ,or bIfUpdateLoader = %d\n", bIfUpdateLoader);
			bNeedClearMisc = false;
		}
	}

	// Otherwise, get ready to boot the main system...
	finish_recovery(NULL);
	ui->ShowText(true);
	ui->Print("All complete successful!please remove the sdcard......\n");
	checkSDRemoved();
	ui->Print("rebooting...\n");

	android_reboot(ANDROID_RB_RESTART, 0, 0);

	return EXIT_SUCCESS;
}

void copy_log_to_sd(){

    //add by kaihui --start
    SetSdcardRootPath();
    ensure_sd_mounted();
    char *sdlogPath = (char *)malloc(100);
    char *kernellogPath = (char *)malloc(100);
    strcpy(sdlogPath, EX_SDCARD_ROOT);
    strcat(sdlogPath, "/recovery.txt");
    strcpy(kernellogPath, EX_SDCARD_ROOT);
    strcat(kernellogPath, "/kernel.txt");
    int i = 0;
    for(i = 10; i >= 0; i--){
        char old_log[150], new_log[150], old_kmsg[100], new_kmsg[100];
        sprintf(old_log, (i == 0) ? "%s" : "%s.%d", sdlogPath, i);
        sprintf(old_kmsg, (i == 0) ? "%s" : "%s.%d", kernellogPath, i);
        sprintf(new_log, "%s.%d", sdlogPath, i+1);
        sprintf(new_kmsg, "%s.%d", kernellogPath, i+1);
        printf("old_log is %s, new_log is %s\n", old_log, new_log);
        if(access(old_log, R_OK) != 0){
            continue ;
        }
        rename(old_log, new_log);
        rename(old_kmsg, new_kmsg);
    }
    redirect_stdio(sdlogPath);
    copy_log_file(TEMPORARY_LOG_FILE, sdlogPath, true);
    //add by kaihui --end
}

int
main(int argc, char **argv) {
    time_t start = time(NULL);

    redirect_stdio(TEMPORARY_LOG_FILE);
    //redirect log to serial output
#ifdef LogToSerial
    freopen("/dev/ttyFIQ0", "a", stdout); setbuf(stdout, NULL);
    freopen("/dev/ttyFIQ0", "a", stderr); setbuf(stderr, NULL);
#endif
    bool bFreeArg=false;
    bool bSDBoot=false;
	bool bUsbBoot=false;

    // If this binary is started with the single argument "--adbd",
    // instead of being the normal recovery binary, it turns into kind
    // of a stripped-down version of adbd that only supports the
    // 'sideload' command.  Note this must be a real argument, not
    // anything in the command file or bootloader control block; the
    // only way recovery should be run with this argument is when it
    // starts a copy of itself from the apply_from_adb() function.
    if (argc == 2 && strcmp(argv[1], "--adbd") == 0) {
        adb_main(0, DEFAULT_ADB_PORT);
        return 0;
    }

    printf("Starting recovery (pid %d) on %s", getpid(), ctime(&start));
	if(check_sdboot()==0) 
		bSDBoot = true;	
	else if(check_usbboot()==0)
		bUsbBoot = true;
    load_volume_table();
    SetSdcardRootPath();
#ifdef LogToSDCard
    copy_log_to_sd();
#endif
/*    for(int n = 0; n < 2; n++) {
        if(0 == ensure_path_mounted(LAST_LOG_FILE)){
            break;
        }else {
            printf("delay 1sec\n");
            sleep(1);
        }
    }
    rotate_logs(KEEP_LOG_COUNT);*/
    LOGD("to dump args befor get_args() : \n");
    dumpCmdArgs(argc, argv, 1);
    if (bSDBoot){
        if(!get_args_from_sd(&argc,&argv,&bFreeArg)){
            printf("get args from sd error\n");
	        get_args(&argc, &argv);
        }
        bNeedClearMisc = false;//don't clear misc command(wipe all)
    }else if (bUsbBoot){
        if(!get_args_from_usb(&argc,&argv,&bFreeArg)){
            printf("get args from usb error\n");
	        get_args(&argc, &argv);
        }
		bNeedClearMisc = false;
    }else {
    	get_args(&argc, &argv);
    }
    LOGD("to dump args after get_args() : \n");
    dumpCmdArgs(argc, argv, 1);



    const char *send_intent = NULL;
    const char *update_package = NULL;
	bool should_wipe_data = false;
    char *sdboot_update_package = NULL;
    bool should_wipe_cache = false;
    bool show_text = false;
    bool sideload = false;
    bool sideload_auto_reboot = false;
    bool just_exit = false;
	char *factory_mode = NULL;
    bool shutdown_after = false;
    bool should_wipe_all = false;
	const char *update_rkimage = NULL;
	char *demo_copy_path = NULL;

    int arg;
    while ((arg = getopt_long(argc, argv, "", OPTIONS, NULL)) != -1) {
        switch (arg) {
        case 'f': factory_mode = optarg; bFactoryMode = true; break;
        case 'i': send_intent = optarg; break;
        case 'u': update_package = optarg; break;
        case 'w': should_wipe_data = true; break;
		case 'k':  update_rkimage = optarg;break;
        case 'c': should_wipe_cache = true; break;
        case 't': show_text = true; break;
        case 's': sideload = true; break;
        case 'a': sideload = true; sideload_auto_reboot = true; break;
        case 'x': just_exit = true; break;
        case 'l': locale = optarg; break;
        case 'g': {
            if (stage == NULL || *stage == '\0') {
                char buffer[20] = "1/";
                strncat(buffer, optarg, sizeof(buffer)-3);
                stage = strdup(buffer);
            }
            break;
        }
        case 'f'+'w': //fw_update
            if((optarg)&&(!sdboot_update_package)){
            sdboot_update_package = strdup(optarg);
            }
            break;
        case 'd': //demo_copy
            if((optarg)&&(! demo_copy_path)){
                demo_copy_path = strdup(optarg);
            }
            break;
        case 'p': shutdown_after = true; break;
        case 'r': reason = optarg; break;
        case 'w'+'a': { should_wipe_all = should_wipe_data = should_wipe_cache = true;show_text = true;} break;
        case '?':
            LOGE("Invalid command argument\n");
            continue;
        }
    }

    if (locale == NULL) {
        load_locale_from_cache();
    }
    printf("locale is [%s]\n", locale);
    printf("stage is [%s]\n", stage);
    printf("reason is [%s]\n", reason);

    Device* device = make_device();
    ui = device->GetUI();
    gCurrentUI = ui;

    ui->SetLocale(locale);
    ui->Init();

    int st_cur, st_max;
    if (stage != NULL && sscanf(stage, "%d/%d", &st_cur, &st_max) == 2) {
        ui->SetStage(st_cur, st_max);
    }

    ui->SetBackground(RecoveryUI::NONE);
    if (show_text) ui->ShowText(true);

    struct selinux_opt seopts[] = {
      { SELABEL_OPT_PATH, "/file_contexts" }
    };

    sehandle = selabel_open(SELABEL_CTX_FILE, seopts, 1);

    if (!sehandle) {
        ui->Print("Warning: No file_contexts\n");
    }

    device->StartRecovery();

    //sdcard may not ready,so wait a feel seconds.
    int i;
    for(i = 0; i < 2; i++) {
		if(0 == ensure_path_mounted(EX_SDCARD_ROOT)){
			break;
		}else {
			printf("delay 2sec\n");
			sleep(2);
		}
	}

    if(bFactoryMode == true){
        sdtool_main(factory_mode, device);
    }

    //boot from sdcard
    if(!check_sdboot() && !sdboot_update_package) {
		printf("find sdfwupdate commandline!\n");
		return sdtool_main(factory_mode, device);
	}else {
		printf("Not enter sdboot!\n");
	}
    SureCacheMount();
    SureMetadataMount();

    //dump ro.bootmode type
    char bootmode[256];
    property_get("ro.bootmode", bootmode, "unknown");
    printf("bootmode = %s \n", bootmode);
    property_get("UserVolumeLabel", gVolume_label, "");

    printf("Command:");
    for (arg = 0; arg < argc; arg++) {
        printf(" \"%s\"", argv[arg]);
    }
    printf("\n");

    if (update_package) {
        // For backwards compatibility on the cache partition only, if
        // we're given an old 'root' path "CACHE:foo", change it to
        // "/cache/foo".
        if (strncmp(update_package, "CACHE:", 6) == 0) {
            int len = strlen(update_package) + 10;
            char* modified_path = (char*)malloc(len);
            strlcpy(modified_path, "/cache/", len);
            strlcat(modified_path, update_package+6, len);
            printf("(replacing path \"%s\" with \"%s\")\n",
                   update_package, modified_path);
            update_package = modified_path;
        }
    }
    printf("\n");
    if (update_rkimage) {
        // For backwards compatibility on the cache partition only, if
        // we're given an old 'root' path "CACHE:foo", change it to
        // "/cache/foo".
        ui->Print("RK image: %s\n", update_rkimage);

    }
    printf("\n");
    property_list(print_property, NULL);
    printf("\n");

    ui->Print("Supported API: %d\n", RECOVERY_API_VERSION);
    int status = INSTALL_SUCCESS;
	

    if (update_package != NULL) {
        status = install_package(update_package, &should_wipe_cache, TEMPORARY_INSTALL_FILE, true);
        if (status == INSTALL_SUCCESS && should_wipe_cache) {
            if (erase_volume("/cache")) {
                LOGE("Cache wipe (requested by package) failed.");
            }
        }
        if (status != INSTALL_SUCCESS) {
            ui->Print("Installation aborted.\n");
            ui->Print("OTA failed! Please power off the device to keep it in this state and file a bug report!\n");

            // If this is an eng or userdebug build, then automatically
            // turn the text display on if the script fails so the error
            // message is visible.
            if (is_ro_debuggable()) {
                ui->ShowText(true);
            }
        }else {
	 		bAutoUpdateComplete=true;
        }
    } else if (update_rkimage != NULL) {
        I("to install rk_img : %s.", update_rkimage);
        status = install_rkimage(update_rkimage);
        if (status != INSTALL_SUCCESS) 
			ui->Print("Installation aborted.\n");
		else 
        	bAutoUpdateComplete=true;

    } else if(sdboot_update_package){
    	printf("bSDBoot = %d, sdboot_update_package=%s\n", bSDBoot, sdboot_update_package);
        if (bSDBoot){
            printf("SDBoot optarg=%s\n", optarg);
            status = do_sd_mode_update(sdboot_update_package);
        } else if(bUsbBoot){
			printf("UsbBoot optarg=%s\n", optarg);
			status = do_usb_mode_update(sdboot_update_package);
        } 
        if (status!=INSTALL_SUCCESS){
            SET_ERROR_AND_JUMP("fail to update from sd.", status, INSTALL_ERROR, HANDLE_STATUS);
        }
        if(demo_copy_path){
            erase_volume(IN_SDCARD_ROOT);
            if (bSDBoot)
                status = do_sd_demo_copy(demo_copy_path);
            else if(bUsbBoot)
                status = do_usb_demo_copy(demo_copy_path);
        
            if (status!=INSTALL_SUCCESS){
                SET_ERROR_AND_JUMP("fail to copy demo.", status, INSTALL_ERROR, HANDLE_STATUS);
            }
		}
    } else if (should_wipe_data) {
        printf("should_wipe_data=%d,should_wipe_cache=%d,wipe_all=%d\n",should_wipe_data,should_wipe_cache,should_wipe_all);
        if (!wipe_data(false, device)) {
            status = INSTALL_ERROR;
        }
        if(should_wipe_all) {
            printf("begin to wipe frp partion!\n");
            int ret;
            ret = format_volume("/frp");
            if(ret<0){
               printf("wiping frp failed!\n");
             } else {
                printf("wiping frp success!\n");
            }
        	printf("resize /system \n");
			Volume* v = volume_for_path("/system");
			if(rk_check_and_resizefs(v->blk_device)) {
				ui->Print("check and resize /system failed!\n");
				status = INSTALL_ERROR;
			}
        }
    } else if (should_wipe_cache) {
		ui->Print("should_wipe_cache\n");
        if (!wipe_cache(false, device)) {
            status = INSTALL_ERROR;
        }
        if (erase_persistent_partition() == -1 ) status = INSTALL_ERROR;
        if (status != INSTALL_SUCCESS) ui->Print("Data wipe failed.\n");
    } else if (sideload) {
        // 'adb reboot sideload' acts the same as user presses key combinations
        // to enter the sideload mode. When 'sideload-auto-reboot' is used, text
        // display will NOT be turned on by default. And it will reboot after
        // sideload finishes even if there are errors. Unless one turns on the
        // text display during the installation. This is to enable automated
        // testing.
        if (!sideload_auto_reboot) {
            ui->ShowText(true);
        }
        status = apply_from_adb(ui, &should_wipe_cache, TEMPORARY_INSTALL_FILE);
        if (status == INSTALL_SUCCESS && should_wipe_cache) {
            if (!wipe_cache(false, device)) {
                status = INSTALL_ERROR;
            }
        }
        ui->Print("\nInstall from ADB complete (status: %d).\n", status);
        if (sideload_auto_reboot) {
            ui->Print("Rebooting automatically.\n");
        }
    } else if (!just_exit) {
        status = INSTALL_NONE;  // No command specified
        ui->SetBackground(RecoveryUI::NO_COMMAND);

        // http://b/17489952
        // If this is an eng or userdebug build, automatically turn on the
        // text display if no command is specified.
        if (is_ro_debuggable()) {
            ui->ShowText(true);
        }
    }

HANDLE_STATUS :
    if (!sideload_auto_reboot && (status == INSTALL_ERROR || status == INSTALL_CORRUPT)) {
        #ifdef LogToCache
        copy_logs();
        #endif
        ui->SetBackground(RecoveryUI::ERROR);
        bNeedClearMisc = true;
    }

    if (bSDBoot){
		ui->ShowText(true);
		if (status==INSTALL_SUCCESS)
	    	ui->Print("Doing Actions succeeded.please remove the sdcard......\n");
		else
	    	ui->Print("Doing Actions failed!please remove the sdcard......\n");
        #ifdef LogToSDCard
        char *kernellogPath = (char *)malloc(100);
        strcpy(kernellogPath, EX_SDCARD_ROOT);
        strcat(kernellogPath, "/kernel.txt");
        save_kernel_log(kernellogPath);
        #endif
		if (bSDMounted)
            checkSDRemoved();

    } else if (bUsbBoot) {
        ui->ShowText(true);
		if (status==INSTALL_SUCCESS)
	    	ui->Print("Doing Actions succeeded.please remove the usb disk......\n");
		else
	    	ui->Print("Doing Actions failed!please remove the usb disk......\n");
		if (bUsbMounted)
	    	checkUSBRemoved();
    }
    Device::BuiltinAction after = shutdown_after ? Device::SHUTDOWN : Device::REBOOT;
    if ((status != INSTALL_SUCCESS && !sideload_auto_reboot)) {
        ui->ShowText(true);
        Device::BuiltinAction temp = prompt_and_wait(device, status);
        if (temp != Device::NO_ACTION) {
            after = temp;
        }
    }

    // Save logs and clean up before rebooting or shutting down.
    finish_recovery(send_intent);

    switch (after) {
        case Device::SHUTDOWN:
            ui->Print("Shutting down...\n");
            property_set(ANDROID_RB_PROPERTY, "shutdown,");
            break;

        case Device::REBOOT_BOOTLOADER:
            ui->Print("Rebooting to bootloader...\n");
            property_set(ANDROID_RB_PROPERTY, "reboot,bootloader");
            break;

        default:
            ui->Print("Rebooting...\n");
            property_set(ANDROID_RB_PROPERTY, "reboot,");
            break;
    }
    sleep(5); // should reboot before this finishes
    return EXIT_SUCCESS;
}
