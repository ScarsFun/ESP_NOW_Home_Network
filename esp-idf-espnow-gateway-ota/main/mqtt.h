#define	PUBLISH		100
#define	SUBSCRIBE	200
#define	STOP		900

typedef struct {
    int topic_type;
    int topic_len;
    char topic[64];
    int data_len;
    char data[180];
} MQTT_t;

void mqtt_init(void);