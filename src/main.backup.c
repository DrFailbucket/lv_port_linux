/*******************************************************************
 *
 * main.c - LVGL Application for Battery Charger with WiFi (DEBUG VERSION)
 *
 * Author: EDGEMTech Ltd, Erik Tagirov (erik.tagirov@edgemtech.ch)
 *
 ******************************************************************/
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>
#include <time.h>
#include <curl/curl.h>

#include "cJSON.h"
#include "lvgl/lvgl.h"
#include "lvgl/demos/lv_demos.h"
#include "ui/ui.h"
#include "src/lib/driver_backends.h"
#include "src/lib/simulator_util.h"
#include "src/lib/simulator_settings.h"

/* ===== Configuration ===== */
#define GUI_DATA_FILE "/home/breuil/gui_data.json"
#define WIFI_DATA_FILE "/home/breuil/wifi_data.json"
#define SAVED_WIFI_FILE "/home/breuil/saved_wifi.json"
#define BATTERY_STATS_FILE "/home/breuil/battery_stats.json"
#define OTA_CONFIG_FILE "/home/breuil/ota_config.json"
#define LOGFILE "/home/breuil/logfile.txt"
#define MAX_MODULES 8

/* OTA Configuration */
#define GITHUB_REPO_OWNER "DrFailbucket"
#define GITHUB_REPO_NAME "PowerDock"
#define CURRENT_VERSION "1.0.3"
#define GITHUB_API_URL "https://api.github.com/repos/" GITHUB_REPO_OWNER "/" GITHUB_REPO_NAME "/releases/latest"

typedef enum {
    LOG_NONE     = 0,
    LOG_STDOUT   = 1 << 0,
    LOG_FILE     = 1 << 1,
    LOG_BOTH     = LOG_STDOUT | LOG_FILE
} LogTarget;

static LogTarget g_log_target = LOG_STDOUT;
static FILE *g_log_file = NULL;

void log_set_target(LogTarget target) {
    g_log_target = target;
}

int log_open_file(const char *filename) {
    if (g_log_file != NULL) {
        fclose(g_log_file);
    }
    g_log_file = fopen(filename, "a");
    return (g_log_file != NULL) ? 0 : -1;
}

void log_close_file(void) {
    if (g_log_file != NULL) {
        fclose(g_log_file);
        g_log_file = NULL;
    }
}

/* ========== ERWEITERT: Log-Levels ========== */

typedef enum {
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR
} LogLevel;

static LogLevel g_min_log_level = LOG_LEVEL_DEBUG;

void log_set_level(LogLevel level) {
    g_min_log_level = level;
}

/* ========== MAKROS ========== */

#define DEBUG_LOG_LEVEL(level, fmt, ...) do { \
    if (level >= g_min_log_level) { \
        time_t now = time(NULL); \
        struct tm *t = localtime(&now); \
        const char *level_str[] = {"DEBUG", "INFO", "WARN", "ERROR"}; \
        char buffer[1024]; \
        snprintf(buffer, sizeof(buffer), "[%02d:%02d:%02d] [%s] " fmt "\n", \
                 t->tm_hour, t->tm_min, t->tm_sec, level_str[level], ##__VA_ARGS__); \
        \
        if (g_log_target & LOG_STDOUT) { \
            fprintf(stdout, "%s", buffer); \
            fflush(stdout); \
        } \
        if ((g_log_target & LOG_FILE) && g_log_file != NULL) { \
            fprintf(g_log_file, "%s", buffer); \
            fflush(g_log_file); \
        } \
    } \
} while(0)

#define LOG_DEBUG_ONCE(fmt, ...) do { \
    static bool _logged = false; \
    if (!_logged) { \
        LOG_DEBUG(fmt, ##__VA_ARGS__); \
        _logged = true; \
    } \
} while(0)

/* Convenience-Makros */
#define LOG_DEBUG(fmt, ...) DEBUG_LOG_LEVEL(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  DEBUG_LOG_LEVEL(LOG_LEVEL_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  DEBUG_LOG_LEVEL(LOG_LEVEL_WARNING, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) DEBUG_LOG_LEVEL(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)

/* Kompatibilitaet: Altes DEBUG_LOG weiter nutzbar */
#define DEBUG_LOG(fmt, ...) LOG_DEBUG(fmt, ##__VA_ARGS__)

/* ===== Global Variables ===== */
static char *selected_backend;
extern simulator_settings_t settings;
static lv_obj_t *wifi_spinner = NULL;
static char *pending_update_version = NULL;

/* ===== Function Prototypes ===== */
static void configure_simulator(int argc, char **argv);
static void print_lvgl_version(void);
static void print_usage(void);

/* System Control */
static void btn_shutdown_cb(lv_event_t *e);
static void btn_reboot_cb(lv_event_t *e);

/* WiFi Management */
static void sync_wlan_toggle_with_service_status(void);
static void ui_event_btnsearch(lv_event_t *e);
static void ui_event_btnConnect(lv_event_t *e);
static void ui_event_savedSSIDs_changed(lv_event_t *e);
static void ui_event_SwitchWLANOnOff(lv_event_t *e);
static void load_saved_ssids_from_file(void);
static void wlan_status_timer(lv_timer_t *t);

/* Battery/JSON Update */
static void json_update_timer(lv_timer_t *t);

/* Helper Functions */
static void show_temp_label(const char *text, lv_color_t color);
static void temp_label_timer_cb(lv_timer_t *timer);
static void wifi_status_popup(void);

/* OTA Functions */
static int check_wifi_connection(void);
static void check_for_updates(void);
static void ui_event_btnCheckUpdates_cb(lv_event_t *e);
static void ui_event_ddOTA_cb(lv_event_t *e);
static void update_check_button_visibility(void);
static char* load_github_token(void);
static void show_update_install_popup(const char *version);
static void install_update_cb(lv_event_t *e);
static void cancel_update_cb(lv_event_t *e);
static void run_update_installation(const char *version);

/* ===== OTA Update Functions ===== */

typedef struct {
    char *data;
    size_t size;
} curl_response_t;

static char* load_github_token(void) {
    LOG_DEBUG_ONCE("OTA: Loading GitHub token from config file: %s", OTA_CONFIG_FILE);
    
    FILE *f = fopen(OTA_CONFIG_FILE, "r");
    if (!f) {
        LOG_INFO("OTA: Config file not found, continuing without authentication");
        return NULL;
    }
    
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    if (fsize <= 0 || fsize > 10000) {
        LOG_WARN("OTA: Invalid config file size: %ld bytes", fsize);
        fclose(f);
        return NULL;
    }
    
    char *json_str = malloc(fsize + 1);
    if (!json_str) {
        LOG_ERROR("OTA: Memory allocation failed for config");
        fclose(f);
        return NULL;
    }
    
    fread(json_str, 1, fsize, f);
    json_str[fsize] = 0;
    fclose(f);
    
    cJSON *root = cJSON_Parse(json_str);
    free(json_str);
    
    if (!root) {
        LOG_WARN("OTA: Failed to parse config JSON");
        return NULL;
    }
    
    cJSON *token_item = cJSON_GetObjectItem(root, "github_token");
    char *token = NULL;
    
    if (cJSON_IsString(token_item) && token_item->valuestring != NULL) {
        token = strdup(token_item->valuestring);
        LOG_INFO("OTA: GitHub token loaded successfully");
    } else {
        LOG_DEBUG("OTA: No github_token field found in config");
    }
    
    cJSON_Delete(root);
    return token;
}

static size_t ota_curl_write_cb(void *contents, size_t size, size_t nmemb, void *userp) {
    size_t realsize = size * nmemb;
    curl_response_t *mem = (curl_response_t *)userp;
    
    char *ptr = realloc(mem->data, mem->size + realsize + 1);
    if(!ptr) {
        LOG_ERROR("OTA: realloc failed in curl callback");
        return 0;
    }
    
    mem->data = ptr;
    memcpy(&(mem->data[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->data[mem->size] = 0;
    
    return realsize;
}

static int check_wifi_connection(void) {
    LOG_DEBUG_ONCE("OTA: Checking WiFi connection...");
    
    FILE *fp = popen("systemctl is-active NetworkManager.service 2>/dev/null", "r");
    if (!fp) {
        LOG_WARN("OTA: Could not check NetworkManager status");
        return 0;
    }
    
    char status[32];
    int nm_active = 0;
    if (fgets(status, sizeof(status), fp) != NULL) {
        status[strcspn(status, "\n")] = 0;
        LOG_DEBUG("OTA: NetworkManager status: %s", status);
        if (strcmp(status, "active") == 0) {
            nm_active = 1;
        }
    }
    pclose(fp);
    
    if (!nm_active) {
        LOG_DEBUG("OTA: NetworkManager not active");
        return 0;
    }
    
    fp = popen("nmcli -t -f STATE general 2>/dev/null", "r");
    if (!fp) {
        LOG_WARN("OTA: Could not check general network state");
        return 0;
    }
    
    char net_state[64];
    int has_connectivity = 0;
    if (fgets(net_state, sizeof(net_state), fp) != NULL) {
        net_state[strcspn(net_state, "\n")] = 0;
        LOG_DEBUG("OTA: General network state: %s", net_state);
        
        if (strstr(net_state, "connected") != NULL) {
            has_connectivity = 1;
        }
    }
    pclose(fp);
    
    if (!has_connectivity) {
        LOG_DEBUG("OTA: No network connectivity");
        return 0;
    }
    
    fp = popen("nmcli -t -f GENERAL.STATE device show wlan0 2>/dev/null", "r");
    if (!fp) {
        LOG_DEBUG("OTA: Could not check wlan0 state, trying route check");
        
        fp = popen("ip route | grep default 2>/dev/null", "r");
        if (!fp) {
            LOG_WARN("OTA: Could not check default route");
            return 0;
        }
        
        char route[256];
        if (fgets(route, sizeof(route), fp) != NULL) {
            LOG_DEBUG("OTA: Default route exists");
            pclose(fp);
            LOG_INFO("OTA: WiFi connected (via route check)");
            return 1;
        }
        pclose(fp);
        return 0;
    }
    
    char device_state[64];
    int wlan_connected = 0;
    if (fgets(device_state, sizeof(device_state), fp) != NULL) {
        device_state[strcspn(device_state, "\n")] = 0;
        LOG_DEBUG("OTA: wlan0 device state: %s", device_state);
        
        if (strstr(device_state, "connected") != NULL || 
            strstr(device_state, "100") != NULL) {
            wlan_connected = 1;
        }
    }
    pclose(fp);
    
    if (wlan_connected) {
        LOG_INFO("OTA: WiFi connected");
    } else {
        LOG_DEBUG("OTA: WiFi not connected");
    }
    
    return wlan_connected;
}

static int is_newer_version(const char *current, const char *latest) {
    int cur_major = 0, cur_minor = 0, cur_patch = 0;
    int lat_major = 0, lat_minor = 0, lat_patch = 0;
    
    sscanf(current, "%d.%d.%d", &cur_major, &cur_minor, &cur_patch);
    sscanf(latest, "%d.%d.%d", &lat_major, &lat_minor, &lat_patch);
    
    if (lat_major > cur_major) return 1;
    if (lat_major == cur_major && lat_minor > cur_minor) return 1;
    if (lat_major == cur_major && lat_minor == cur_minor && lat_patch > cur_patch) return 1;
    
    return 0;
}

static void check_for_updates(void) {
    LOG_INFO("OTA: Checking for updates on GitHub...");
    LOG_DEBUG("OTA: Repository: %s/%s", GITHUB_REPO_OWNER, GITHUB_REPO_NAME);
    LOG_DEBUG("OTA: API URL: %s", GITHUB_API_URL);
    
    char *github_token = load_github_token();
    
    CURL *curl;
    CURLcode res;
    curl_response_t response = {0};
    
    curl = curl_easy_init();
    if(!curl) {
        LOG_ERROR("OTA: curl_easy_init failed");
        show_temp_label("OTA: curl init failed", lv_color_hex(0xFF0000));
        if (github_token) free(github_token);
        return;
    }
    
    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "User-Agent: PowerDock-OTA");
    headers = curl_slist_append(headers, "Accept: application/vnd.github.v3+json");
    
    char auth_header[512];
    if (github_token) {
        snprintf(auth_header, sizeof(auth_header), "Authorization: token %s", github_token);
        headers = curl_slist_append(headers, auth_header);
        LOG_DEBUG("OTA: Using authentication token");
    } else {
        LOG_DEBUG("OTA: No token - accessing as public repo");
    }
    
    curl_easy_setopt(curl, CURLOPT_URL, GITHUB_API_URL);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, ota_curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&response);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "PowerDock-OTA/1.0");
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
    
    res = curl_easy_perform(curl);
    
    if (github_token) {
        free(github_token);
    }
    
    if(res != CURLE_OK) {
        LOG_ERROR("OTA: curl_easy_perform failed: %s", curl_easy_strerror(res));
        show_temp_label("OTA: Connection failed", lv_color_hex(0xFF0000));
        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
        if(response.data) free(response.data);
        return;
    }
    
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    LOG_DEBUG("OTA: HTTP response code: %ld", http_code);
    
    if (response.data && response.size > 0) {
        LOG_DEBUG("OTA: Response size: %zu bytes", response.size);
        LOG_DEBUG_ONCE("OTA: Response preview: %.200s", response.data);
    } else {
        LOG_WARN("OTA: No response data received");
    }
    
    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);
    
    if(http_code == 401) {
        LOG_ERROR("OTA: Authentication failed - check your GitHub token");
        show_temp_label("OTA: Auth failed", lv_color_hex(0xFF0000));
        if(response.data) free(response.data);
        return;
    }
    
    if(http_code == 404) {
        LOG_WARN("OTA: GitHub API returned HTTP 404");
        LOG_DEBUG("OTA: Possible causes:");
        LOG_DEBUG("OTA:   1. Repository is private and no token provided");
        LOG_DEBUG("OTA:   2. Repository name is incorrect");
        LOG_DEBUG("OTA:   3. No releases exist");
        LOG_DEBUG("OTA:   4. Release is saved as draft or pre-release");
        show_temp_label("OTA: No releases found", lv_color_hex(0xFF0000));
        if(response.data) free(response.data);
        return;
    }
    
    if(http_code != 200) {
        LOG_ERROR("OTA: GitHub API returned HTTP %ld", http_code);
        show_temp_label("OTA: API error", lv_color_hex(0xFF0000));
        if(response.data) free(response.data);
        return;
    }
    
    cJSON *root = cJSON_Parse(response.data);
    free(response.data);
    
    if (!root) {
        LOG_ERROR("OTA: JSON parse failed");
        show_temp_label("OTA: Parse error", lv_color_hex(0xFF0000));
        return;
    }
    
    cJSON *tag_name = cJSON_GetObjectItem(root, "tag_name");
    
    if (cJSON_IsString(tag_name)) {
        const char *latest_version = tag_name->valuestring;
        
        if (latest_version[0] == 'v') {
            latest_version++;
        }
        
        LOG_INFO("OTA: Current version: %s", CURRENT_VERSION);
        LOG_INFO("OTA: Latest version: %s", latest_version);
        
        if (is_newer_version(CURRENT_VERSION, latest_version)) {
            LOG_INFO("OTA: Update available!");
            show_update_install_popup(latest_version);
        } else {
            LOG_INFO("OTA: Software is up to date");
            show_temp_label("Software is up to date", lv_color_hex(0x0080FF));
        }
    } else {
        LOG_ERROR("OTA: tag_name not found in response");
        show_temp_label("OTA: Invalid response", lv_color_hex(0xFF0000));
    }
    
    cJSON_Delete(root);
}

static void update_check_button_visibility(void) {
    if (!ui_btnCheckUpdates || !lv_obj_is_valid(ui_btnCheckUpdates)) {
        LOG_DEBUG_ONCE("OTA: btnCheckUpdates not found");
        return;
    }
    
    if (!ui_ddOTA || !lv_obj_is_valid(ui_ddOTA)) {
        LOG_DEBUG_ONCE("OTA: ddOTA not found");
        return;
    }
    
    uint16_t selected = lv_dropdown_get_selected(ui_ddOTA);
    
    if (selected == 1) {
        LOG_DEBUG("OTA: Showing Check Updates button");
        lv_obj_clear_flag(ui_btnCheckUpdates, LV_OBJ_FLAG_HIDDEN);
    } else {
        LOG_DEBUG("OTA: Hiding Check Updates button");
        lv_obj_add_flag(ui_btnCheckUpdates, LV_OBJ_FLAG_HIDDEN);
    }
}

static void ui_event_ddOTA_cb(lv_event_t *e) {
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    
    LOG_DEBUG("OTA: Dropdown value changed");
    update_check_button_visibility();
}

static void ui_event_btnCheckUpdates_cb(lv_event_t *e) {
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    
    LOG_INFO("OTA: Check Updates button clicked");
    
    if (!check_wifi_connection()) {
        LOG_WARN("OTA: No WiFi connection");
        show_temp_label("No WiFi connection", lv_color_hex(0xFF0000));
        return;
    }
    
    show_temp_label("Checking for updates...", lv_color_hex(0x0080FF));
    check_for_updates();
}

static void show_update_install_popup(const char *version) {
    LOG_INFO("OTA: Showing install popup for version %s", version);
    
    if (pending_update_version) {
        free(pending_update_version);
    }
    pending_update_version = strdup(version);
    
    lv_obj_t *popup = lv_obj_create(lv_scr_act());
    lv_obj_set_size(popup, 400, 200);
    lv_obj_center(popup);
    lv_obj_set_style_bg_color(popup, lv_color_hex(0x1BAAF5), 0);
    lv_obj_set_style_bg_opa(popup, LV_OPA_100, 0);
    lv_obj_set_style_border_width(popup, 2, 0);
    lv_obj_set_style_border_color(popup, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(popup, 10, 0);
    lv_obj_set_style_shadow_width(popup, 20, 0);
    lv_obj_set_style_shadow_spread(popup, 2, 0);
    
    lv_obj_t *label_question = lv_label_create(popup);
    char msg[128];
    snprintf(msg, sizeof(msg), "Install update v%s?", version);
    lv_label_set_text(label_question, msg);
    lv_obj_set_style_text_font(label_question, &lv_font_montserrat_24, 0);
    lv_obj_align(label_question, LV_ALIGN_TOP_MID, 0, 20);
    
    lv_obj_t *label_info = lv_label_create(popup);
    lv_label_set_text(label_info, "This will download and install\nthe update automatically.");
    lv_obj_set_style_text_font(label_info, &lv_font_montserrat_14, 0);
    lv_obj_align(label_info, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_align(label_info, LV_TEXT_ALIGN_CENTER, 0);
    
    lv_obj_t *btn_install = lv_button_create(popup);
    lv_obj_set_size(btn_install, 120, 50);
    lv_obj_align(btn_install, LV_ALIGN_BOTTOM_LEFT, 30, -20);
    lv_obj_set_style_bg_color(btn_install, lv_color_hex(0x00FF00), 0);
    lv_obj_add_event_cb(btn_install, install_update_cb, LV_EVENT_CLICKED, popup);
    
    lv_obj_t *label_install = lv_label_create(btn_install);
    lv_label_set_text(label_install, "Install");
    lv_obj_set_style_text_font(label_install, &lv_font_montserrat_18, 0);
    lv_obj_center(label_install);
    
    lv_obj_t *btn_cancel = lv_button_create(popup);
    lv_obj_set_size(btn_cancel, 120, 50);
    lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_RIGHT, -30, -20);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_hex(0xFF0000), 0);
    lv_obj_add_event_cb(btn_cancel, cancel_update_cb, LV_EVENT_CLICKED, popup);
    
    lv_obj_t *label_cancel = lv_label_create(btn_cancel);
    lv_label_set_text(label_cancel, "Cancel");
    lv_obj_set_style_text_font(label_cancel, &lv_font_montserrat_18, 0);
    lv_obj_center(label_cancel);
    
    LOG_DEBUG("OTA: Install popup created");
}

static void install_update_cb(lv_event_t *e) {
    LOG_INFO("OTA: Install button clicked");
    
    lv_obj_t *popup = (lv_obj_t *)lv_event_get_user_data(e);
    
    if (pending_update_version) {
        LOG_INFO("OTA: Starting installation of version %s", pending_update_version);
        show_temp_label("Installing update...", lv_color_hex(0xFFFF00));
        run_update_installation(pending_update_version);
        free(pending_update_version);
        pending_update_version = NULL;
    }
    
    if (popup && lv_obj_is_valid(popup)) {
        lv_obj_del(popup);
    }
}

static void cancel_update_cb(lv_event_t *e) {
    LOG_INFO("OTA: Cancel button clicked");
    
    lv_obj_t *popup = (lv_obj_t *)lv_event_get_user_data(e);
    
    if (pending_update_version) {
        free(pending_update_version);
        pending_update_version = NULL;
    }
    
    if (popup && lv_obj_is_valid(popup)) {
        lv_obj_del(popup);
    }
    
    show_temp_label("Update cancelled", lv_color_hex(0xFF0000));
}

static void run_update_installation(const char *version) {
    LOG_INFO("OTA: Running update installation for version %s", version);
    
    char cmd[512];
    snprintf(cmd, sizeof(cmd), 
             "python3 /home/breuil/ota_install.py %s %s %s &", 
             GITHUB_REPO_OWNER, GITHUB_REPO_NAME, version);
    
    LOG_DEBUG("OTA: Executing: %s", cmd);
    
    int ret = system(cmd);
    
    if (ret == 0) {
        LOG_INFO("OTA: Update installation started successfully");
        show_temp_label("Update started - check logs", lv_color_hex(0x00FF00));
    } else {
        LOG_ERROR("OTA: Failed to start update installation (code: %d)", ret);
        show_temp_label("Update failed to start", lv_color_hex(0xFF0000));
    }
}

/* ===== System Control Callbacks ===== */

static void btn_shutdown_cb(lv_event_t *e) {
    if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
        LOG_WARN("Shutdown button pressed - executing shutdown");
        system("sudo shutdown -h now");
    }
}

static void btn_reboot_cb(lv_event_t *e) {
    if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
        LOG_WARN("Reboot button pressed - executing reboot");
        system("sudo reboot");
    }
}

/* ===== WiFi Status Helpers ===== */

static void sync_wlan_toggle_with_service_status(void) {
    LOG_DEBUG_ONCE("sync_wlan_toggle_with_service_status: Syncing toggle state");
    
    if (ui_Label35 && !lv_obj_check_type(ui_Label35, &lv_label_class)) {
        LOG_WARN("ui_Label35 is not a label!");
        return;
    }
    
    FILE *fp = popen("systemctl is-active NetworkManager.service", "r");
    if (!fp) {
        LOG_ERROR("Could not check NetworkManager status");
        return;
    }
    
    char status[32];
    if (fgets(status, sizeof(status), fp) != NULL) {
        status[strcspn(status, "\n")] = 0;
        LOG_DEBUG("NetworkManager status: %s", status);
        
        if (strcmp(status, "active") == 0) {
            if (ui_SwitchWLANOnOff && lv_obj_is_valid(ui_SwitchWLANOnOff)) {
                lv_obj_add_state(ui_SwitchWLANOnOff, LV_STATE_CHECKED);
            } else {
                LOG_ERROR("ui_SwitchWLANOnOff is NULL or invalid!");
            }
            if (ui_Label35 && lv_obj_is_valid(ui_Label35)) {
                lv_label_set_text(ui_Label35, "WLAN aktiv");
            }
        } else {
            if (ui_SwitchWLANOnOff && lv_obj_is_valid(ui_SwitchWLANOnOff)) {
                lv_obj_clear_state(ui_SwitchWLANOnOff, LV_STATE_CHECKED);
            }
            if (ui_Label35 && lv_obj_is_valid(ui_Label35)) {
                lv_label_set_text(ui_Label35, "WLAN inaktiv");
            }
        }
    }
    
    pclose(fp);
}

static void temp_label_timer_cb(lv_timer_t *timer) {
    lv_obj_t *lbl = (lv_obj_t *)lv_timer_get_user_data(timer);
    if (lbl && lv_obj_is_valid(lbl)) {
        lv_obj_del(lbl);
    }
}

static void wlan_status_timer(lv_timer_t *t) {
    LV_UNUSED(t);
    sync_wlan_toggle_with_service_status();
}

static void show_temp_label(const char *text, lv_color_t color) {
    LOG_DEBUG_ONCE("show_temp_label: Creating temporary label");
    
    lv_obj_t *label = lv_label_create(lv_scr_act());
    if (!label) {
        LOG_ERROR("Could not create temp label!");
        return;
    }
    
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_bg_color(label, lv_color_hex(0x202020), 0);
    lv_obj_set_style_bg_opa(label, LV_OPA_80, 0);
    lv_obj_set_style_pad_all(label, 6, 0);
    lv_obj_set_style_radius(label, 6, 0);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -10);

    lv_timer_t *t = lv_timer_create_basic();
    lv_timer_set_period(t, 10000);
    lv_timer_set_repeat_count(t, 1);
    lv_timer_set_user_data(t, label);
    lv_timer_set_cb(t, temp_label_timer_cb);
}

static void wifi_status_popup(void) {
    LOG_DEBUG_ONCE("wifi_status_popup: Checking WiFi status");
    
    FILE *fp = popen("nmcli -t -f ACTIVE,SSID,SIGNAL dev wifi", "r");
    if (!fp) {
        LOG_ERROR("Could not run nmcli");
        show_temp_label("WiFi check failed", lv_color_hex(0xFF0000));
        return;
    }
    
    char line[256];
    int found = 0;
    while (fgets(line, sizeof(line), fp)) {
        if (strncmp(line, "yes:", 4) == 0) {
            found = 1;
            char *p = line + 4;
            char *ssid = strtok(p, ":");
            char *signal = strtok(NULL, ":");
            if (!ssid) ssid = (char *)"unknown";
            if (!signal) signal = (char *)"0";
            
            char msg[256];
            snprintf(msg, sizeof(msg), "Connected: %s (%s%%)", ssid, signal);
            LOG_INFO("WiFi connected: %s (%s%%)", ssid, signal);
            show_temp_label(msg, lv_color_hex(0x00FF00));
            break;
        }
    }
    pclose(fp);
    
    if (!found) {
        LOG_INFO("No active WiFi connection found");
        show_temp_label("No WiFi connection", lv_color_hex(0xFF0000));
    }
}
/* ===== WiFi List Management ===== */

static void update_wifi_list(void) {
    LOG_DEBUG_ONCE("update_wifi_list: Updating WiFi dropdown");
    
    FILE *f = fopen(WIFI_DATA_FILE, "r");
    if (!f) {
        LOG_ERROR("Could not open %s", WIFI_DATA_FILE);
        return;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    LOG_DEBUG_ONCE("WiFi data file size: %ld bytes", fsize);

    char *json_str = malloc(fsize + 1);
    fread(json_str, 1, fsize, f);
    fclose(f);
    json_str[fsize] = 0;

    cJSON *root = cJSON_Parse(json_str);
    free(json_str);

    if (!root) {
        LOG_ERROR("Could not parse WiFi JSON");
        return;
    }

    cJSON *ssids = cJSON_GetObjectItem(root, "ssids");
    if (cJSON_IsArray(ssids)) {
        char options[2048] = "";
        int count = cJSON_GetArraySize(ssids);
        LOG_INFO("Found %d WiFi networks", count);

        for (int i = 0; i < count; i++) {
            cJSON *ssid = cJSON_GetArrayItem(ssids, i);
            if (cJSON_IsString(ssid)) {
                const char *ssid_str = cJSON_GetStringValue(ssid);
                LOG_DEBUG_ONCE("SSID[%d]: %s", i, ssid_str);
                strcat(options, ssid_str);
                if (i < count - 1) strcat(options, "\n");
            }
        }

        if (ui_ssidList && lv_obj_is_valid(ui_ssidList)) {
            lv_dropdown_clear_options(ui_ssidList);
            lv_dropdown_set_options(ui_ssidList, options);
            LOG_DEBUG("Dropdown updated successfully");
        } else {
            LOG_ERROR("ui_ssidList is NULL or invalid!");
        }
    }

    cJSON_Delete(root);
}

static void load_saved_ssids_from_file(void) {
    LOG_DEBUG_ONCE("load_saved_ssids_from_file: Loading saved WiFi connections");
    
    FILE *f = fopen(SAVED_WIFI_FILE, "r");
    if(!f) {
        LOG_INFO("No saved WiFi file found");
        if (ui_savedSSIDs && lv_obj_is_valid(ui_savedSSIDs)) {
            lv_dropdown_set_options(ui_savedSSIDs, "Keine gespeicherten Netzwerke");
        }
        return;
    }
    
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    LOG_DEBUG_ONCE("Saved WiFi file size: %ld bytes", fsize);
    
    char *json_str = malloc(fsize + 1);
    fread(json_str, 1, fsize, f);
    fclose(f);
    json_str[fsize] = 0;
    
    cJSON *root = cJSON_Parse(json_str);
    free(json_str);
    
    if(root) {
        cJSON *saved = cJSON_GetObjectItem(root, "saved_connections");
        if(cJSON_IsArray(saved)) {
            char options[2048] = "";
            int count = cJSON_GetArraySize(saved);
            LOG_INFO("Loaded %d saved connections", count);
            
            if(count == 0) {
                strcpy(options, "Keine gespeicherten Netzwerke");
            } else {
                for(int i = 0; i < count; i++) {
                    cJSON *item = cJSON_GetArrayItem(saved, i);
                    if(cJSON_IsObject(item)) {
                        cJSON *display = cJSON_GetObjectItem(item, "display_name");
                        if(cJSON_IsString(display)) {
                            const char *name = cJSON_GetStringValue(display);
                            LOG_DEBUG_ONCE("Saved connection[%d]: %s", i, name);
                            strcat(options, name);
                            if(i < count - 1) strcat(options, "\n");
                        }
                    }
                }
            }
            
            if (ui_savedSSIDs && lv_obj_is_valid(ui_savedSSIDs)) {
                lv_dropdown_set_options(ui_savedSSIDs, options);
                LOG_DEBUG("Saved SSIDs dropdown updated");
            } else {
                LOG_ERROR("ui_savedSSIDs is NULL or invalid!");
            }
        }
        cJSON_Delete(root);
    }
}

static void update_saved_ssids(void) {
    LOG_DEBUG_ONCE("update_saved_ssids: Running Python script");
    system("python3 /home/breuil/saved_wifi.py");
    load_saved_ssids_from_file();
}

static void delayed_load_saved_ssids_timer(lv_timer_t *t) {
    (void)t;
    update_saved_ssids();
    lv_timer_del(t);
}

static void delayed_update_saved_ssids(lv_timer_t *t) {
    (void)t;
    update_saved_ssids();
    lv_timer_del(t);
}

/* ===== WiFi Event Callbacks ===== */

static void wifi_spinner_timer_cb(lv_timer_t *timer) {
    LV_UNUSED(timer);
    if (wifi_spinner && lv_obj_is_valid(wifi_spinner)) {
        LOG_DEBUG("Deleting WiFi spinner");
        lv_obj_del(wifi_spinner);
        wifi_spinner = NULL;
    }
    update_wifi_list();
}

static void ui_event_btnsearch(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    LOG_INFO("WiFi Search started");

    if (wifi_spinner) {
        LOG_DEBUG("Spinner already active, ignoring click");
        return;
    }

    wifi_spinner = lv_spinner_create(lv_scr_act());
    if (!wifi_spinner) {
        LOG_ERROR("Could not create spinner!");
        return;
    }
    
    lv_obj_set_size(wifi_spinner, 100, 100);
    lv_obj_center(wifi_spinner);
    lv_spinner_set_anim_params(wifi_spinner, 1000, 60);

    show_temp_label("WiFi search started", lv_color_hex(0x00FF00));

    LOG_DEBUG("Starting Python WiFi scan script");
    system("python3 /home/breuil/wifi_scan.py &");

    lv_timer_t *t = lv_timer_create_basic();
    lv_timer_set_period(t, 12000);
    lv_timer_set_repeat_count(t, 1);
    lv_timer_set_user_data(t, NULL);
    lv_timer_set_cb(t, wifi_spinner_timer_cb);
}

static void ui_event_btnConnect(lv_event_t *e) {
    if(lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    char ssid[128];
    
    if (!ui_ssidList || !lv_obj_is_valid(ui_ssidList)) {
        LOG_ERROR("ui_ssidList is NULL or invalid!");
        return;
    }
    
    lv_dropdown_get_selected_str(ui_ssidList, ssid, sizeof(ssid));
    
    if (!ui_InputPassKey || !lv_obj_is_valid(ui_InputPassKey)) {
        LOG_ERROR("ui_InputPassKey is NULL or invalid!");
        return;
    }
    
    const char *pass = lv_textarea_get_text(ui_InputPassKey);

    LOG_INFO("WiFi Connect: SSID='%s', password length=%zu", ssid, strlen(pass));

    char cmd[512];
    snprintf(cmd, sizeof(cmd),
        "sudo nmcli dev wifi connect '%s' password '%s' > /dev/null 2>&1",
        ssid, pass);

    if (ui_Label35 && lv_obj_is_valid(ui_Label35)) {
        lv_label_set_text(ui_Label35, "Verbinde...");
    }
    lv_refr_now(NULL);
    
    LOG_DEBUG("Executing nmcli connect command");
    int ret = system(cmd);
    LOG_DEBUG("nmcli returned: %d", ret);
    
    lv_obj_invalidate(lv_scr_act());
    lv_refr_now(NULL);
    
    if (ret == 0) {
        LOG_INFO("WiFi connected successfully");
        if (ui_Label35 && lv_obj_is_valid(ui_Label35)) {
            lv_label_set_text(ui_Label35, "Verbunden!");
        }
        show_temp_label("WiFi connected", lv_color_hex(0x00FF00));
        wifi_status_popup();
        
        LOG_DEBUG("Waiting 2 seconds before updating saved SSIDs");
        usleep(2000000);
        update_saved_ssids();
        
    } else {
        LOG_ERROR("WiFi connection failed (code %d)", ret);
        if (ui_Label35 && lv_obj_is_valid(ui_Label35)) {
            lv_label_set_text(ui_Label35, "Verbindung fehlgeschlagen");
        }
        show_temp_label("Connection failed", lv_color_hex(0xFF0000));
    }
}

static void ui_event_savedSSIDs_changed(lv_event_t *e) {
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    lv_obj_t *dropdown = lv_event_get_target(e);
    char display_name[128];
    lv_dropdown_get_selected_str(dropdown, display_name, sizeof(display_name));
    LOG_DEBUG("Selected saved SSID: %s", display_name);

    if (strstr(display_name, "Keine gespeicherten") != NULL) {
        LOG_DEBUG("No saved networks option selected, ignoring");
        return;
    }

    FILE *f = fopen(SAVED_WIFI_FILE, "r");
    if(!f) {
        LOG_ERROR("Could not open saved WiFi file");
        return;
    }
    
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *json_str = malloc(fsize + 1);
    fread(json_str, 1, fsize, f);
    fclose(f);
    json_str[fsize] = 0;
    
    cJSON *root = cJSON_Parse(json_str);
    free(json_str);
    
    char conn_name[256] = "";
    if(root) {
        cJSON *saved = cJSON_GetObjectItem(root, "saved_connections");
        uint16_t idx = lv_dropdown_get_selected(dropdown);
        LOG_DEBUG("Dropdown index: %u", idx);
        
        if(cJSON_IsArray(saved) && idx < cJSON_GetArraySize(saved)) {
            cJSON *item = cJSON_GetArrayItem(saved, idx);
            if(cJSON_IsObject(item)) {
                cJSON *name = cJSON_GetObjectItem(item, "connection_name");
                if(cJSON_IsString(name)) {
                    strncpy(conn_name, cJSON_GetStringValue(name), sizeof(conn_name)-1);
                    LOG_DEBUG("Connection name: %s", conn_name);
                }
            }
        }
        cJSON_Delete(root);
    }
    
    if(conn_name[0] == '\0') {
        LOG_ERROR("Connection name not found");
        show_temp_label("Connection not found", lv_color_hex(0xFF0000));
        return;
    }
    
    LOG_INFO("Auto-connecting to saved WiFi: %s", conn_name);
    LOG_DEBUG("Disconnecting wlan0");
    system("sudo nmcli dev disconnect wlan0 > /dev/null 2>&1");
    usleep(1000000);

    char cmd[512];
    snprintf(cmd, sizeof(cmd),
             "sudo nmcli connection up '%s' ifname wlan0 > /dev/null 2>&1",
             conn_name);
    
    LOG_DEBUG("Executing: %s", cmd);
    int ret = system(cmd);
    LOG_DEBUG("nmcli returned: %d", ret);

    lv_obj_invalidate(lv_scr_act());
    lv_refr_now(NULL);
    
    if (ret == 0) {
        LOG_INFO("WiFi connected successfully");
        if (ui_Label35 && lv_obj_is_valid(ui_Label35)) {
            lv_label_set_text(ui_Label35, "Verbunden");
        }
        show_temp_label("WiFi connected", lv_color_hex(0x00FF00));
        wifi_status_popup();
    } else {
        LOG_ERROR("WiFi connection failed (code %d)", ret);
        if (ui_Label35 && lv_obj_is_valid(ui_Label35)) {
            lv_label_set_text(ui_Label35, "Fehler bei Verbindung");
        }
        show_temp_label("Connection failed", lv_color_hex(0xFF0000));
    }
}

static void ui_event_SwitchWLANOnOff(lv_event_t *e) {
    if(lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    lv_obj_t *sw = lv_event_get_target(e);
    bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
    LOG_INFO("WLAN toggle: %s", state ? "ON" : "OFF");

    lv_obj_t *spinner = lv_spinner_create(lv_scr_act());
    if (!spinner) {
        LOG_ERROR("Could not create spinner!");
        return;
    }
    
    lv_obj_set_size(spinner, 80, 80);
    lv_obj_center(spinner);
    lv_spinner_set_anim_params(spinner, 1000, 60);
    lv_refr_now(NULL);

    if (state) {
        LOG_INFO("WiFi Toggle ON - Starting services...");
        show_temp_label("WLAN wird aktiviert...", lv_color_hex(0xFFFF00));
        
        LOG_DEBUG("Starting wpa_supplicant");
        system("sudo systemctl start wpa_supplicant.service");
        usleep(500000);
        lv_refr_now(NULL);
        
        LOG_DEBUG("Starting NetworkManager");
        system("sudo systemctl start NetworkManager.service");
        usleep(500000);
        lv_refr_now(NULL);
        
        LOG_DEBUG("Enabling WiFi radio");
        system("sudo nmcli radio wifi on");
        usleep(1000000);
        
        if (ui_Label35 && lv_obj_is_valid(ui_Label35)) {
            lv_label_set_text(ui_Label35, "WLAN aktiviert");
        }
        show_temp_label("WiFi enabled", lv_color_hex(0x00FF00));
        
        wifi_status_popup();
        
        LOG_DEBUG("Creating delayed timer for SSID update");
        lv_timer_t *delayed = lv_timer_create(delayed_update_saved_ssids, 3000, NULL);
        lv_timer_set_repeat_count(delayed, 1);
        
    } else {
        LOG_INFO("WiFi Toggle OFF - Stopping services...");
        show_temp_label("WLAN wird deaktiviert...", lv_color_hex(0xFFFF00));
        
        LOG_DEBUG("Disabling WiFi radio");
        system("sudo nmcli radio wifi off");
        usleep(500000);
        lv_refr_now(NULL);
        
        LOG_DEBUG("Stopping NetworkManager");
        system("sudo systemctl stop NetworkManager.service");
        usleep(500000);
        lv_refr_now(NULL);
        
        LOG_DEBUG("Stopping wpa_supplicant");
        system("sudo systemctl stop wpa_supplicant.service");
        usleep(500000);
        
        if (ui_Label35 && lv_obj_is_valid(ui_Label35)) {
            lv_label_set_text(ui_Label35, "WLAN deaktiviert");
        }
        show_temp_label("WiFi disabled", lv_color_hex(0xFF0000));
    }
    
    if (spinner && lv_obj_is_valid(spinner)) {
        lv_obj_del(spinner);
    }
    lv_obj_invalidate(lv_scr_act());
    lv_refr_now(NULL);
    
    lv_timer_t *status_refresh = lv_timer_create_basic();
    lv_timer_set_period(status_refresh, 2000);
    lv_timer_set_repeat_count(status_refresh, 1);
    lv_timer_set_cb(status_refresh, wlan_status_timer);
}

/* ===== Battery Statistics Display ===== */

static int current_battery_id = -1;

static void update_battery_info_panel(int battery_id) {
    LOG_DEBUG_ONCE("update_battery_info_panel: First call for battery %d", battery_id);
    
    if (battery_id < 0 || battery_id >= MAX_MODULES) {
        LOG_ERROR("Invalid battery ID: %d", battery_id);
        return;
    }
    
    current_battery_id = battery_id;
    
    FILE *f = fopen(BATTERY_STATS_FILE, "r");
    if (!f) {
        LOG_WARN("Could not open %s", BATTERY_STATS_FILE);
        if (ui_lbTotalChargingTime && lv_obj_is_valid(ui_lbTotalChargingTime))
            lv_label_set_text(ui_lbTotalChargingTime, "N/A");
        if (ui_lbWh && lv_obj_is_valid(ui_lbWh))
            lv_label_set_text(ui_lbWh, "N/A");
        if (ui_lbAh && lv_obj_is_valid(ui_lbAh))
            lv_label_set_text(ui_lbAh, "N/A");
        if (ui_lbMinTemp && lv_obj_is_valid(ui_lbMinTemp))
            lv_label_set_text(ui_lbMinTemp, "N/A");
        if (ui_lbMaxTemp && lv_obj_is_valid(ui_lbMaxTemp))
            lv_label_set_text(ui_lbMaxTemp, "N/A");
        if (ui_lbSoH && lv_obj_is_valid(ui_lbSoH))
            lv_label_set_text(ui_lbSoH, "N/A");
        if (ui_lbSoC && lv_obj_is_valid(ui_lbSoC))
            lv_label_set_text(ui_lbSoC, "N/A");
        return;
    }
    
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    LOG_DEBUG_ONCE("Battery stats file size: %ld bytes", fsize);
    
    char *buf = malloc(fsize + 1);
    fread(buf, 1, fsize, f);
    buf[fsize] = '\0';
    fclose(f);
    
    cJSON *root = cJSON_Parse(buf);
    free(buf);
    
    if (!root) {
        LOG_ERROR("Could not parse battery stats JSON");
        return;
    }
    
    cJSON *modules = cJSON_GetObjectItem(root, "modules");
    if (!cJSON_IsArray(modules)) {
        LOG_ERROR("modules is not an array in stats JSON");
        cJSON_Delete(root);
        return;
    }
    
    cJSON *module = NULL;
    cJSON *m;
    cJSON_ArrayForEach(m, modules) {
        cJSON *id = cJSON_GetObjectItem(m, "id");
        if (cJSON_IsNumber(id) && id->valueint == battery_id) {
            module = m;
            LOG_DEBUG_ONCE("Found module data for battery %d", battery_id);
            break;
        }
    }
    
    if (!module) {
        LOG_WARN("Module %d not found in stats", battery_id);
        cJSON_Delete(root);
        return;
    }
    
    cJSON *total_time = cJSON_GetObjectItem(module, "total_charging_time");
    cJSON *wh = cJSON_GetObjectItem(module, "wh");
    cJSON *ah = cJSON_GetObjectItem(module, "ah");
    cJSON *min_temp = cJSON_GetObjectItem(module, "min_temp");
    cJSON *max_temp = cJSON_GetObjectItem(module, "max_temp");
    cJSON *soh = cJSON_GetObjectItem(module, "soh");
    cJSON *soc = cJSON_GetObjectItem(module, "soc");
    
    char text[64];
    
    if (cJSON_IsNumber(total_time)) {
        int seconds = total_time->valueint;
        int hours = seconds / 3600;
        int minutes = (seconds % 3600) / 60;
        int secs = seconds % 60;
        snprintf(text, sizeof(text), "%02d:%02d:%02d", hours, minutes, secs);
        if (ui_lbTotalChargingTime && lv_obj_is_valid(ui_lbTotalChargingTime)) {
            lv_label_set_text(ui_lbTotalChargingTime, text);
        }
    }
    
    if (cJSON_IsNumber(wh)) {
        snprintf(text, sizeof(text), "%.2f Wh", wh->valuedouble);
        if (ui_lbWh && lv_obj_is_valid(ui_lbWh)) {
            lv_label_set_text(ui_lbWh, text);
        }
    }
    
    if (cJSON_IsNumber(ah)) {
        snprintf(text, sizeof(text), "%.3f Ah", ah->valuedouble);
        if (ui_lbAh && lv_obj_is_valid(ui_lbAh)) {
            lv_label_set_text(ui_lbAh, text);
        }
    }
    
    if (cJSON_IsNumber(min_temp)) {
        snprintf(text, sizeof(text), "%.1f C", min_temp->valuedouble);
        if (ui_lbMinTemp && lv_obj_is_valid(ui_lbMinTemp)) {
            lv_label_set_text(ui_lbMinTemp, text);
        }
    }
    
    if (cJSON_IsNumber(max_temp)) {
        snprintf(text, sizeof(text), "%.1f C", max_temp->valuedouble);
        if (ui_lbMaxTemp && lv_obj_is_valid(ui_lbMaxTemp)) {
            lv_label_set_text(ui_lbMaxTemp, text);
        }
    }
    
    if (cJSON_IsNumber(soh)) {
        snprintf(text, sizeof(text), "%.1f %%", soh->valuedouble);
        if (ui_lbSoH && lv_obj_is_valid(ui_lbSoH)) {
            lv_label_set_text(ui_lbSoH, text);
        }
    }
    
    if (cJSON_IsNumber(soc)) {
        snprintf(text, sizeof(text), "%.1f %%", soc->valuedouble);
        if (ui_lbSoC && lv_obj_is_valid(ui_lbSoC)) {
            lv_label_set_text(ui_lbSoC, text);
        }
    }
    
    cJSON_Delete(root);
    LOG_DEBUG_ONCE("Battery %d info updated successfully", battery_id);
}

static void battery_info_refresh_timer(lv_timer_t *t) {
    (void)t;
    
    if (current_battery_id >= 0 && ui_Panel2 && lv_obj_is_valid(ui_Panel2) && 
        !lv_obj_has_flag(ui_Panel2, LV_OBJ_FLAG_HIDDEN)) {
        update_battery_info_panel(current_battery_id);
    }
}

static void battery_info_btn_cb(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    
    lv_obj_t *btn = lv_event_get_target(e);
    int battery_id = -1;
    
    if (btn == ui_btnInfoBatt1) battery_id = 0;
    else if (btn == ui_btnInfoBatt2) battery_id = 1;
    else if (btn == ui_btnInfoBatt3) battery_id = 2;
    else if (btn == ui_btnInfoBatt4) battery_id = 3;
    else if (btn == ui_btnInfoBatt5) battery_id = 4;
    else if (btn == ui_btnInfoBatt6) battery_id = 5;
    else if (btn == ui_btnInfoBatt7) battery_id = 6;
    else if (btn == ui_btnInfoBatt8) battery_id = 7;
    
    LOG_INFO("Battery info button pressed: ID=%d", battery_id);
    
    if (battery_id >= 0) {
        update_battery_info_panel(battery_id);
    }
}

/* ===== Battery Data Update ===== */

static void update_from_json(void) {
    static bool was_successful = true;
    static time_t last_error_log = 0;
    static int consecutive_errors = 0;
    
    LOG_DEBUG_ONCE("=== update_from_json: First call snapshot ===");
    LOG_DEBUG_ONCE("GUI_DATA_FILE path: %s", GUI_DATA_FILE);
    
    struct stat st;
    if (stat(GUI_DATA_FILE, &st) != 0) {
        if (was_successful) {
            LOG_ERROR("GUI data file not found: %s", GUI_DATA_FILE);
            was_successful = false;
        }
        return;
    }
    
    if (st.st_size < 50) {
        return;
    }
    
    LOG_DEBUG_ONCE("File exists, size: %ld bytes", st.st_size);
    
    FILE *f = fopen(GUI_DATA_FILE, "r");
    if (!f) {
        if (was_successful) {
            LOG_ERROR("Could not open file: %s", GUI_DATA_FILE);
            was_successful = false;
        }
        return;
    }
    LOG_DEBUG_ONCE("File opened successfully");
    
    char *buf = malloc(st.st_size + 1);
    if (!buf) {
        LOG_ERROR("malloc failed for %ld bytes", st.st_size + 1);
        fclose(f);
        return;
    }
    LOG_DEBUG_ONCE("Allocated %ld bytes for buffer", st.st_size + 1);
    
    size_t bytes_read = fread(buf, 1, st.st_size, f);
    buf[st.st_size] = '\0';
    fclose(f);
    LOG_DEBUG_ONCE("Read %zu bytes from file", bytes_read);
    LOG_DEBUG_ONCE("File content preview: %.500s", buf);
    
    LOG_DEBUG_ONCE("Attempting to parse JSON...");
    cJSON *root = cJSON_Parse(buf);
    
    if (!root) {
        consecutive_errors++;
        
        time_t now = time(NULL);
        if ((was_successful && consecutive_errors == 1) || 
            ((now - last_error_log) > 10 && consecutive_errors > 20)) {
            
            const char *error_ptr = cJSON_GetErrorPtr();
            LOG_WARN("JSON parse errors detected (%d consecutive)", consecutive_errors);
            if (error_ptr != NULL) {
                LOG_DEBUG("JSON error before: %.50s", error_ptr);
            }
            LOG_DEBUG("This usually happens during file write operations");
            last_error_log = now;
            consecutive_errors = 0;
        }
        
        was_successful = false;
        free(buf);
        return;
    }
    
    if (!was_successful || consecutive_errors > 0) {
        if (consecutive_errors > 5) {
            LOG_INFO("JSON parsing recovered after %d errors", consecutive_errors);
        }
        was_successful = true;
        consecutive_errors = 0;
    }
    
    free(buf);
    LOG_DEBUG_ONCE("JSON parsed successfully");
    
    cJSON *mods = cJSON_GetObjectItem(root, "modules");
    if (!mods) {
        if (was_successful) {
            LOG_ERROR("'modules' key not found in JSON");
            was_successful = false;
        }
        cJSON_Delete(root);
        return;
    }
    LOG_DEBUG_ONCE("Found 'modules' key");
    
    if (!cJSON_IsArray(mods)) {
        if (was_successful) {
            LOG_ERROR("'modules' is not an array (type: %d)", mods->type);
            was_successful = false;
        }
        cJSON_Delete(root);
        return;
    }
    
    int array_size = cJSON_GetArraySize(mods);
    LOG_DEBUG_ONCE("modules array size: %d", array_size);
    
    lv_obj_t *labels[MAX_MODULES] = {
        ui_VoltLabel1, ui_VoltLabel2, ui_VoltLabel3, ui_VoltLabel4,
        ui_VoltLabel5, ui_VoltLabel6, ui_VoltLabel7, ui_VoltLabel8
    };
    lv_obj_t *bars[MAX_MODULES] = {
        ui_VoltBar1, ui_VoltBar2, ui_VoltBar3, ui_VoltBar4,
        ui_VoltBar5, ui_VoltBar6, ui_VoltBar7, ui_VoltBar8
    };
    lv_obj_t *voltage_labels[MAX_MODULES] = {
        ui_VoltLabel9,  ui_VoltLabel10, ui_VoltLabel11, ui_VoltLabel12,
        ui_VoltLabel13, ui_VoltLabel14, ui_VoltLabel15, ui_VoltLabel16
    };
    
    LOG_DEBUG_ONCE("UI element pointers initialized");
    
    int idx = 0;
    int updated_count = 0;
    cJSON *m;
    
    cJSON_ArrayForEach(m, mods) {
        if (idx >= MAX_MODULES) {
            LOG_WARN("Reached MAX_MODULES limit (%d), ignoring remaining modules", MAX_MODULES);
            break;
        }
        
        cJSON *bus_v = cJSON_GetObjectItem(m, "bus_voltage");
        if (!bus_v) {
            LOG_DEBUG_ONCE("Module %d: 'bus_voltage' key not found", idx);
            idx++;
            continue;
        }
        
        if (!cJSON_IsNumber(bus_v)) {
            LOG_DEBUG_ONCE("Module %d: bus_voltage is not a number", idx);
            idx++;
            continue;
        }
        
        float voltage = bus_v->valuedouble;
        LOG_DEBUG_ONCE("Module %d: First voltage reading = %.2f V", idx, voltage);
        
        static time_t last_voltage_warn[MAX_MODULES] = {0};
        time_t now = time(NULL);
        
        if (voltage < 18.0f && (now - last_voltage_warn[idx]) > 60) {
            LOG_WARN("Module %d: Low voltage %.2f V (below 18V)", idx, voltage);
            last_voltage_warn[idx] = now;
        }
        if (voltage > 21.0f && (now - last_voltage_warn[idx]) > 60) {
            LOG_WARN("Module %d: High voltage %.2f V (above 21V)", idx, voltage);
            last_voltage_warn[idx] = now;
        }
        
        int percent = (int)(((voltage - 18.0f) / (21.0f - 18.0f)) * 100.0f);
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
        
        char txt[16];
        snprintf(txt, sizeof(txt), "%d", percent);
        
        if (bars[idx] && lv_obj_is_valid(bars[idx])) {
            lv_bar_set_value(bars[idx], percent, LV_ANIM_ON);
        } else {
            LOG_DEBUG_ONCE("Module %d: Bar is NULL or invalid", idx);
        }
        
        if (labels[idx] && lv_obj_is_valid(labels[idx])) {
            lv_label_set_text(labels[idx], txt);
        } else {
            LOG_DEBUG_ONCE("Module %d: Percent label is NULL or invalid", idx);
        }
        
        char voltage_txt[16];
        snprintf(voltage_txt, sizeof(voltage_txt), "%.1f", voltage);
        if (voltage_labels[idx] && lv_obj_is_valid(voltage_labels[idx])) {
            lv_label_set_text(voltage_labels[idx], voltage_txt);
        } else {
            LOG_DEBUG_ONCE("Module %d: Voltage label is NULL or invalid", idx);
        }
        
        updated_count++;
        idx++;
    }
    
    LOG_DEBUG_ONCE("Updated %d/%d modules (first time)", updated_count, array_size);
    
    cJSON_Delete(root);
    LOG_DEBUG_ONCE("=== update_from_json: First call completed ===");
}

static void json_update_timer(lv_timer_t *t) {
    LV_UNUSED(t);
    update_from_json();
}

/* ===== Simulator Configuration ===== */

static void print_lvgl_version(void) {
    LOG_INFO("%d.%d.%d-%s",
            LVGL_VERSION_MAJOR,
            LVGL_VERSION_MINOR,
            LVGL_VERSION_PATCH,
            LVGL_VERSION_INFO);
}

static void print_usage(void) {
    LOG_INFO("lvglsim [-V] [-B] [-b backend_name] [-W width] [-H height]");
    LOG_INFO("-V print LVGL version");
    LOG_INFO("-B list supported backends");
}

static void configure_simulator(int argc, char **argv) {
    LOG_DEBUG_ONCE("configure_simulator: Configuring simulator");
    
    int opt = 0;
    selected_backend = NULL;
    driver_backends_register();

    const char *env_w = getenv("LV_SIM_WINDOW_WIDTH");
    const char *env_h = getenv("LV_SIM_WINDOW_HEIGHT");
    settings.window_width = atoi(env_w ? env_w : "480");
    settings.window_height = atoi(env_h ? env_h : "320");
    
    LOG_DEBUG_ONCE("Window size: %dx%d", settings.window_width, settings.window_height);

    while ((opt = getopt(argc, argv, "b:W:H:BVh")) != -1) {
        switch (opt) {
        case 'h':
            print_usage();
            exit(EXIT_SUCCESS);
        case 'V':
            print_lvgl_version();
            exit(EXIT_SUCCESS);
        case 'B':
            driver_backends_print_supported();
            exit(EXIT_SUCCESS);
        case 'b':
            if (driver_backends_is_supported(optarg) == 0) {
                die("error no such backend: %s\n", optarg);
            }
            selected_backend = strdup(optarg);
            LOG_DEBUG("Selected backend: %s", selected_backend);
            break;
        case 'W':
            settings.window_width = atoi(optarg);
            LOG_DEBUG("Window width: %d", settings.window_width);
            break;
        case 'H':
            settings.window_height = atoi(optarg);
            LOG_DEBUG("Window height: %d", settings.window_height);
            break;
        case ':':
        case '?':
            print_usage();
            die("Unknown option or missing argument\n");
        }
    }
}

static void lvgl_log_cb(lv_log_level_t level, const char *buf) {
    (void)level;
    LOG_DEBUG("[LVGL] %s", buf);
}

/* ===== Main Function ===== */

int main(int argc, char **argv) {
    /* --- Logging Setup --- */
    log_open_file(LOGFILE);
    
    /* Ausgabeziel waehlen (eine Zeile auskommentieren): */
    // log_set_target(LOG_STDOUT);   /* Nur Terminal */
    // log_set_target(LOG_FILE);     /* Nur Logdatei */
    log_set_target(LOG_BOTH);        /* Terminal + Logdatei */
    
    log_set_level(LOG_LEVEL_DEBUG);
    
    LOG_INFO("=== LVGL Application Starting ===");
    LOG_DEBUG("argc=%d", argc);
    for (int i = 0; i < argc; i++) {
        LOG_DEBUG("argv[%d]=%s", i, argv[i]);
    }

    /* --- Initialisierung --- */
    curl_global_init(CURL_GLOBAL_DEFAULT);
    configure_simulator(argc, argv);
    lv_init();

    lv_log_register_print_cb(lvgl_log_cb);

    LOG_INFO("Initializing display backend: %s", selected_backend ? selected_backend : "default");
    if (driver_backends_init_backend(selected_backend) == -1) {
        LOG_ERROR("FATAL: Failed to initialize display backend");
        curl_global_cleanup();
        die("Failed to initialize display backend");
    }

    #if LV_USE_EVDEV
        LOG_INFO("Initializing EVDEV");
        if (driver_backends_init_backend("EVDEV") == -1) {
            LOG_ERROR("FATAL: Failed to initialize evdev");
            curl_global_cleanup();
            die("Failed to initialize evdev");
        }
    #endif

    LOG_INFO("Initializing UI");
    ui_init();
    usleep(200000);

    /* --- Timer Setup --- */
    LOG_DEBUG("Creating timers");
    lv_timer_create(json_update_timer, 500, NULL);
    lv_timer_create(battery_info_refresh_timer, 2000, NULL);

    lv_timer_t *init_timer = lv_timer_create_basic();
    lv_timer_set_period(init_timer, 1000);
    lv_timer_set_repeat_count(init_timer, 1);
    lv_timer_set_cb(init_timer, wlan_status_timer);

    lv_timer_t *delayed_timer = lv_timer_create(delayed_load_saved_ssids_timer, 5000, NULL);
    lv_timer_set_repeat_count(delayed_timer, 1);

    /* --- WLAN Setup --- */
    LOG_INFO("Loading saved WiFi connections");
    load_saved_ssids_from_file();

    /* --- Event Handler Registrierung --- */
    LOG_DEBUG("Registering event handlers");

    /* System */
    lv_obj_add_event_cb(ui_btnShutdown, btn_shutdown_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_btnReboot, btn_reboot_cb, LV_EVENT_CLICKED, NULL);

    /* WiFi */
    lv_obj_add_event_cb(ui_btnsearch, ui_event_btnsearch, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_btnConnect, ui_event_btnConnect, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_savedSSIDs, ui_event_savedSSIDs_changed, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui_SwitchWLANOnOff, ui_event_SwitchWLANOnOff, LV_EVENT_VALUE_CHANGED, NULL);

    /* Battery info */
    lv_obj_add_event_cb(ui_btnInfoBatt1, battery_info_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_btnInfoBatt2, battery_info_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_btnInfoBatt3, battery_info_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_btnInfoBatt4, battery_info_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_btnInfoBatt5, battery_info_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_btnInfoBatt6, battery_info_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_btnInfoBatt7, battery_info_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_btnInfoBatt8, battery_info_btn_cb, LV_EVENT_CLICKED, NULL);

    /* OTA */
    LOG_DEBUG("Registering OTA event handlers");
    if (ui_btnCheckUpdates && lv_obj_is_valid(ui_btnCheckUpdates)) {
        lv_obj_add_event_cb(ui_btnCheckUpdates, ui_event_btnCheckUpdates_cb, LV_EVENT_ALL, NULL);
        LOG_DEBUG("  - ui_btnCheckUpdates registered");
    } else {
        LOG_ERROR("  - ERROR: ui_btnCheckUpdates not found!");
    }

    if (ui_ddOTA && lv_obj_is_valid(ui_ddOTA)) {
        lv_obj_add_event_cb(ui_ddOTA, ui_event_ddOTA_cb, LV_EVENT_ALL, NULL);
        LOG_DEBUG("  - ui_ddOTA registered");
    } else {
        LOG_ERROR("  - ERROR: ui_ddOTA not found!");
    }

    /* --- WLAN Sync --- */
    LOG_DEBUG("Syncing WLAN toggle state");
    sync_wlan_toggle_with_service_status();

    /* --- OTA Startup Check --- */
    LOG_INFO("=== OTA Startup Check ===");
    update_check_button_visibility();

    if (check_wifi_connection()) {
        LOG_INFO("OTA: WiFi connected at startup");
        if (ui_ddOTA && lv_obj_is_valid(ui_ddOTA)) {
            uint16_t selected = lv_dropdown_get_selected(ui_ddOTA);
            if (selected == 1) {
                LOG_INFO("OTA: Enabled, checking for updates...");
                check_for_updates();
            } else {
                LOG_INFO("OTA: Disabled in settings");
            }
        }
    } else {
        LOG_WARN("OTA: No WiFi connection, skipping update check");
    }

    /* --- Run Loop --- */
    LOG_INFO("=== Entering run loop ===");
    driver_backends_run_loop();

    /* --- Cleanup --- */
    LOG_INFO("=== Application exiting ===");
    log_close_file();
    curl_global_cleanup();

    return 0;
}