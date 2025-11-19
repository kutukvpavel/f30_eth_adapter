#include "eth_console_vfs.h"

#include "eth_console.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdio_ext.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"
#include "esp_check.h"

// Token signifying that no character is available
#define NONE -1

#define FD_CHECK(fd, ret_val) \
    do                        \
    {                         \
        if ((fd) != 0)        \
        {                     \
            errno = EBADF;    \
            return (ret_val); \
        }                     \
    } while (0)

#if CONFIG_NEWLIB_STDOUT_LINE_ENDING_CRLF
#define DEFAULT_TX_MODE ESP_LINE_ENDINGS_CRLF
#elif CONFIG_NEWLIB_STDOUT_LINE_ENDING_CR
#define DEFAULT_TX_MODE ESP_LINE_ENDINGS_CR
#else
#define DEFAULT_TX_MODE ESP_LINE_ENDINGS_LF
#endif

#if CONFIG_NEWLIB_STDIN_LINE_ENDING_CRLF
#define DEFAULT_RX_MODE ESP_LINE_ENDINGS_CRLF
#elif CONFIG_NEWLIB_STDIN_LINE_ENDING_CR
#define DEFAULT_RX_MODE ESP_LINE_ENDINGS_CR
#else
#define DEFAULT_RX_MODE ESP_LINE_ENDINGS_LF
#endif

#define STRINGIFY(s) STRINGIFY2(s)
#define STRINGIFY2(s) #s

#define VFS_ETH_MAX_PATH 16
#define VFS_ETH_PATH_DEFAULT "/dev/ethcon0"

namespace eth_console_vfs
{
    static esp_err_t _register(RingbufHandle_t rx, RingbufHandle_t tx, const char *path);
    static esp_err_t unregister(char const *path);

    const static char *TAG = "eth_console_vfs";
    static FILE* vprintf_stdout = NULL;

    typedef struct
    {
        _lock_t write_lock;
        _lock_t read_lock;
        esp_line_endings_t tx_mode; // Newline conversion mode when transmitting
        esp_line_endings_t rx_mode; // Newline conversion mode when receiving
        volatile uint32_t flags;
        char vfs_path[VFS_ETH_MAX_PATH];
        RingbufHandle_t buffer_rx = NULL;
        RingbufHandle_t buffer_tx = NULL;
        bool registered = false;
        int peek_char = NONE;
    } vfs_eth_t;

    static vfs_eth_t s_vfseth = {};

    esp_err_t init_console()
    {
        ESP_RETURN_ON_ERROR(esp_eth_console_create(&s_vfseth.buffer_rx, &s_vfseth.buffer_tx), TAG, "Failed to initialize eth console");
        ESP_RETURN_ON_ERROR(_register(s_vfseth.buffer_rx, s_vfseth.buffer_tx, NULL), TAG, "Failed to register eth console in VFS");
        vprintf_stdout = fopen(s_vfseth.vfs_path, "w");
        assert(vprintf_stdout);
        return ESP_OK;
    }

    esp_err_t deinit_console()
    {
        unregister(NULL);
        return ESP_OK;
    }

    static esp_err_t apply_path(char const *path)
    {
        if (path == NULL)
        {
            path = VFS_ETH_PATH_DEFAULT;
        }

        size_t path_len = strlen(path) + 1;
        if (path_len > VFS_ETH_MAX_PATH)
        {
            ESP_LOGE(TAG, "The path is too long; maximum is %d characters", VFS_ETH_MAX_PATH);
            return ESP_ERR_INVALID_ARG;
        }
        strncpy(s_vfseth.vfs_path, path, (VFS_ETH_MAX_PATH - 1));
        ESP_LOGV(TAG, "Path is set to `%s`", path);
        return ESP_OK;
    }

    static esp_err_t vfseth_init(RingbufHandle_t rx, RingbufHandle_t tx, char const *path)
    {
        s_vfseth.buffer_rx = rx;
        s_vfseth.buffer_tx = tx;
        s_vfseth.tx_mode = DEFAULT_TX_MODE;
        s_vfseth.rx_mode = DEFAULT_RX_MODE;
        s_vfseth.flags = 0;
        return apply_path(path);
    }

    /**
     * @brief Clear s_vfseth to default values
     */
    static void vfseth_deinit(void)
    {
        memset(&s_vfseth, 0, sizeof(s_vfseth));
    }

    static int eth_open(const char *path, int flags, int mode)
    {
        (void)mode;
        (void)path;
        s_vfseth.flags = flags;
        return 0;
    }

    static ssize_t eth_write(int fd, const void *data, size_t size)
    {
        FD_CHECK(fd, -1);
        size_t written_sz = 0;
        const char *data_c = (const char *)data;
        _lock_acquire(&(s_vfseth.write_lock));
        for (size_t i = 0; i < size; i++)
        {
            char c = data_c[i];
            if (c != '\n')
            {
                if (xRingbufferSend(s_vfseth.buffer_tx, &c, sizeof(c), (s_vfseth.flags & O_NONBLOCK) ? 0 : portMAX_DELAY) == pdFALSE)
                {
                    break; // can't write anymore
                }
            }
            else
            {
                if (s_vfseth.tx_mode == ESP_LINE_ENDINGS_CRLF || s_vfseth.tx_mode == ESP_LINE_ENDINGS_CR)
                {
                    char cr = '\r';
                    if (xRingbufferSend(s_vfseth.buffer_tx, &cr, sizeof(cr), (s_vfseth.flags & O_NONBLOCK) ? 0 : portMAX_DELAY) == pdFALSE)
                    {
                        break; // can't write anymore
                    }
                }
                if (s_vfseth.tx_mode == ESP_LINE_ENDINGS_CRLF || s_vfseth.tx_mode == ESP_LINE_ENDINGS_LF)
                {
                    char lf = '\n';
                    if (xRingbufferSend(s_vfseth.buffer_tx, &lf, sizeof(lf), (s_vfseth.flags & O_NONBLOCK) ? 0 : portMAX_DELAY) == pdFALSE)
                    {
                        break; // can't write anymore
                    }
                }
            }
            written_sz++;
        }
        _lock_release(&(s_vfseth.write_lock));
        return written_sz;
    }

    static int eth_close(int fd)
    {
        FD_CHECK(fd, -1);
        return 0;
    }

    static int eth_fsync(int fd)
    {
        FD_CHECK(fd, -1);
        size_t itemsWaiting = 1;
        _lock_acquire(&(s_vfseth.write_lock));
        while (1)
        {
            vRingbufferGetInfo(s_vfseth.buffer_tx, NULL, NULL, NULL, NULL, &itemsWaiting);
            if (itemsWaiting == 0) break;
            vTaskDelay(1);
        }
        _lock_release(&(s_vfseth.write_lock));
        return 0;
    }

    /* Push back a character; it will be returned by next call to uart_read_char */
    static void eth_return_char(int fd, int c)
    {
        assert(s_vfseth.peek_char == NONE);
        s_vfseth.peek_char = c;
    }
    static int eth_read_char(int fd)
    {
        /* return character from peek buffer, if it is there */
        if (s_vfseth.peek_char != NONE) {
            int c = s_vfseth.peek_char;
            s_vfseth.peek_char = NONE;
            return c;
        }
        char* ptr;
        size_t sz;
        while (((ptr = (char*)xRingbufferReceiveUpTo(s_vfseth.buffer_rx, &sz, (s_vfseth.flags & O_NONBLOCK) ? 0 : pdMS_TO_TICKS(20), sizeof(char))) == NULL)
            && !(s_vfseth.flags & O_NONBLOCK)) vTaskDelay(1);
        char c = ptr ? *ptr : NONE;
        if (ptr) vRingbufferReturnItem(s_vfseth.buffer_rx, ptr);
        return c;
    }
    static ssize_t eth_read(int fd, void *data, size_t size)
    {
        FD_CHECK(fd, -1);
        char *data_c = (char *)data;
        size_t received = 0;
        size_t available_size = 0;
        int c = NONE; // store the read char
        _lock_acquire(&(s_vfseth.read_lock));

        if (!(s_vfseth.flags & O_NONBLOCK))
        {
            c = eth_read_char(fd);
        }

        // find the actual fetch size
        vRingbufferGetInfo(s_vfseth.buffer_rx, NULL, NULL, NULL, NULL, &available_size);
        if (c != NONE)
        {
            available_size++;
        }
        if (s_vfseth.peek_char != NONE)
        {
            available_size++;
        }
        size_t fetch_size = MIN(available_size, size);

        if (fetch_size > 0)
        {
            do
            {
                if (c == NONE)
                { // for non-O_NONBLOCK mode, there is already a pre-fetched char
                    c = eth_read_char(fd);
                }
                assert(c != NONE);

                if (c == '\r')
                {
                    if (s_vfseth.rx_mode == ESP_LINE_ENDINGS_CR)
                    {
                        c = '\n';
                    }
                    else if (s_vfseth.rx_mode == ESP_LINE_ENDINGS_CRLF)
                    {
                        /* look ahead */
                        int c2 = eth_read_char(fd);
                        fetch_size--;
                        if (c2 == NONE)
                        {
                            /* could not look ahead, put the current character back */
                            eth_return_char(fd, c);
                            c = NONE;
                            break;
                        }
                        if (c2 == '\n')
                        {
                            /* this was \r\n sequence. discard \r, return \n */
                            c = '\n';
                        }
                        else
                        {
                            /* \r followed by something else. put the second char back,
                             * it will be processed on next iteration. return \r now.
                             */
                            eth_return_char(fd, c2);
                            fetch_size++;
                        }
                    }
                }

                data_c[received] = (char)c;
                ++received;
                c = NONE;
            } while (received < fetch_size);
        }

        if (c != NONE)
        { // fetched, but not used
            eth_return_char(fd, c);
        }
        _lock_release(&(s_vfseth.read_lock));
        if (received > 0)
        {
            return received;
        }
        errno = EWOULDBLOCK;
        return -1;
    }

    static int eth_fstat(int fd, struct stat *st)
    {
        FD_CHECK(fd, -1);
        memset(st, 0, sizeof(*st));
        st->st_mode = S_IFCHR;
        return 0;
    }

    static int eth_fcntl(int fd, int cmd, int arg)
    {
        FD_CHECK(fd, -1);
        int result = 0;
        switch (cmd)
        {
        case F_GETFL:
            result = s_vfseth.flags;
            break;
        case F_SETFL:
            s_vfseth.flags = arg;
            break;
        default:
            result = -1;
            errno = ENOSYS;
            break;
        }
        return result;
    }

    esp_err_t unregister(char const *path)
    {
        if (!s_vfseth.registered)
        {
            ESP_LOGE(TAG, "ETH-VFS not registered! Nothing to unregister.");
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGD(TAG, "Unregistering ETH-VFS driver");
        int res;

        if (path == NULL)
        { // NULL means using the default path for unregistering: VFS_eth_PATH_DEFAULT
            path = VFS_ETH_PATH_DEFAULT;
        }
        res = strcmp(s_vfseth.vfs_path, path);

        if (res)
        {
            res = ESP_ERR_INVALID_ARG;
            ESP_LOGE(TAG, "There is no ETH-VFS driver registered to path '%s' (err: 0x%x)", path, res);
            return res;
        }

        res = esp_vfs_unregister(s_vfseth.vfs_path);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Can't unregister ETH-VFS driver from '%s' (err: 0x%x)", s_vfseth.vfs_path, res);
        }
        else
        {
            ESP_LOGD(TAG, "Unregistered ETH-VFS driver");
            vfseth_deinit();
        }

        return res;
    }

    esp_err_t _register(RingbufHandle_t rx, RingbufHandle_t tx, char const *path)
    {
        if (s_vfseth.registered)
        {
            ESP_LOGE(TAG, "ETH-VFS driver already registered!");
            return ESP_ERR_INVALID_STATE;
        }
        assert(rx);
        assert(tx);
        
        ESP_ERROR_CHECK(vfseth_init(rx, tx, path));

        esp_vfs_t vfs = {
            .flags = ESP_VFS_FLAG_DEFAULT,
            .write = &eth_write,
            .read = &eth_read,
            .open = &eth_open,
            .close = &eth_close,
            .fstat = &eth_fstat,
            .fcntl = &eth_fcntl,
            .fsync = &eth_fsync,
        };
        ESP_ERROR_CHECK(esp_vfs_register(s_vfseth.vfs_path, &vfs, NULL));
        s_vfseth.registered = true;

        return ESP_OK;
    }

    void set_rx_line_endings(esp_line_endings_t mode)
    {
        assert(s_vfseth.registered);

        _lock_acquire(&(s_vfseth.read_lock));
        s_vfseth.rx_mode = mode;
        _lock_release(&(s_vfseth.read_lock));
    }

    void set_tx_line_endings(esp_line_endings_t mode)
    {
        assert(s_vfseth.registered);

        _lock_acquire(&(s_vfseth.write_lock));
        s_vfseth.tx_mode = mode;
        _lock_release(&(s_vfseth.write_lock));
    }

    void discard_input_buffer()
    {
        assert(s_vfseth.registered);
        
        size_t b;
        void* p;
        _lock_acquire(&(s_vfseth.read_lock));
        while ((p = xRingbufferReceive(s_vfseth.buffer_rx, &b, 0)) != NULL)
            vRingbufferReturnItem(s_vfseth.buffer_rx, p);
        _lock_release(&(s_vfseth.read_lock));
    }

    esp_err_t redirect_std_streams()
    {
        if (!s_vfseth.registered) return ESP_ERR_INVALID_STATE;
        freopen(s_vfseth.vfs_path, "r", stdin);
        freopen(s_vfseth.vfs_path, "w", stdout);
        freopen(s_vfseth.vfs_path, "w", stderr);
        return ESP_OK;
    }

    int vprintf(const char* fmt, va_list args)
    {
        if (!vprintf_stdout) return -1;
        return vfprintf(vprintf_stdout, fmt, args);
    }
}
