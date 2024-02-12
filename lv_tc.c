/*********************
 *      INCLUDES
 *********************/

#include "lv_tc.h"

#include "math.h"

#ifdef ESP_PLATFORM
    #include "esp_log.h"
#endif

/**********************
 *      DEFINES
 *********************/
#define TAG "lv_tc"

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void lv_tc_indev_drv_read_cb(lv_indev_drv_t *indevDrv, lv_indev_data_t *data);


/**********************
 *  STATIC VARIABLES
 **********************/

static lv_tc_coeff_t calibResult = {false, 0, 0, 0, 0, 0, 0};

static lv_obj_t *registeredTCScreen = NULL;
static bool (*registeredInputCb)(lv_obj_t *screenObj, lv_indev_data_t *data) = NULL;
static void (*registeredSaveCb)(lv_tc_coeff_t coeff) = NULL;


/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_tc_indev_drv_init(lv_indev_drv_t *indevDrv, void (*readCb)(lv_indev_drv_t *indevDrv, lv_indev_data_t *data)) {
    lv_indev_drv_init(indevDrv);
    indevDrv->type = LV_INDEV_TYPE_POINTER;
    indevDrv->read_cb = lv_tc_indev_drv_read_cb;
    indevDrv->user_data = readCb;
}

void _lv_tc_register_input_cb(lv_obj_t *screenObj, bool (*inputCb)(lv_obj_t *screenObj, lv_indev_data_t *data)) {
    registeredTCScreen = screenObj;
    registeredInputCb = inputCb;
}

void lv_tc_register_coeff_save_cb(void (*saveCb)(lv_tc_coeff_t coeff)) {
    registeredSaveCb = saveCb;
}

lv_tc_coeff_t* lv_tc_get_coeff() {
    return &calibResult;
}

void lv_tc_set_coeff(lv_tc_coeff_t coeff, bool save) {
    calibResult = coeff;

    if(save) {
        lv_tc_save_coeff();
    }
}

#if defined CONFIG_USE_CUSTOM_LV_TC_COEFFICIENTS
void lv_tc_load_coeff_from_config() {
    lv_tc_coeff_t coeff = {
            true,
            atoff(CONFIG_LV_TC_COEFFICIENT_A),
            atoff(CONFIG_LV_TC_COEFFICIENT_B),
            atoff(CONFIG_LV_TC_COEFFICIENT_C),
            atoff(CONFIG_LV_TC_COEFFICIENT_D),
            atoff(CONFIG_LV_TC_COEFFICIENT_E),
            atoff(CONFIG_LV_TC_COEFFICIENT_F)
    };
    lv_tc_set_coeff(coeff, false);
}
#endif

void lv_tc_save_coeff() {
    if(registeredSaveCb) {
        registeredSaveCb(calibResult);
    }
}

void lv_tc_compute_coeff(lv_point_t *scrP, lv_point_t *tchP, bool save) {   //The computation is explained here: https://www.maximintegrated.com/en/design/technical-documents/app-notes/5/5296.html
    const int8_t shift = 8;
    const int32_t divisor = (
          (int32_t)tchP[0].x * ((int32_t)tchP[2].y - (int32_t)tchP[1].y) 
        - (int32_t)tchP[1].x * (int32_t)tchP[2].y 
        + (int32_t)tchP[1].y * (int32_t)tchP[2].x 
        + (int32_t)tchP[0].y * ((int32_t)tchP[1].x - (int32_t)tchP[2].x)
    );

    lv_tc_coeff_t result = {
        true,
        (((int32_t)scrP[0].x * ((int32_t)tchP[2].y - (int32_t)tchP[1].y) - (int32_t)scrP[1].x * (int32_t)tchP[2].y + (int32_t)scrP[2].x * (int32_t)tchP[1].y + ((int32_t)scrP[1].x - (int32_t)scrP[2].x) * (int32_t)tchP[0].y)<<shift) / divisor,
        ((- ((int32_t)scrP[0].x * ((int32_t)tchP[2].x - (int32_t)tchP[1].x) - (int32_t)scrP[1].x * (int32_t)tchP[2].x + (int32_t)scrP[2].x * (int32_t)tchP[1].x + ((int32_t)scrP[1].x - (int32_t)scrP[2].x) * (int32_t)tchP[0].x))<<shift) / divisor,
        (((int32_t)scrP[0].x * ((int32_t)tchP[1].y * (int32_t)tchP[2].x - (int32_t)tchP[1].x * (int32_t)tchP[2].y) + (int32_t)tchP[0].x * ((int32_t)scrP[1].x * (int32_t)tchP[2].y - (int32_t)scrP[2].x * (int32_t)tchP[1].y) + (int32_t)tchP[0].y * ((int32_t)scrP[2].x * (int32_t)tchP[1].x - (int32_t)scrP[1].x * (int32_t)tchP[2].x))<<shift) / divisor,
        (((int32_t)scrP[0].y * ((int32_t)tchP[2].y - (int32_t)tchP[1].y) - (int32_t)scrP[1].y * (int32_t)tchP[2].y + (int32_t)scrP[2].y * (int32_t)tchP[1].y + ((int32_t)scrP[1].y - (int32_t)scrP[2].y) * (int32_t)tchP[0].y)<<shift) / divisor,
        ((- (
              (int32_t)scrP[0].y * ((int32_t)tchP[2].x - (int32_t)tchP[1].x)
            - (int32_t)scrP[1].y * (int32_t)tchP[2].x
            + (int32_t)scrP[2].y * (int32_t)tchP[1].x
            + ((int32_t)scrP[1].y - (int32_t)scrP[2].y) * (int32_t)tchP[0].x
        ))<<shift) / divisor,
        ((
              (int32_t)scrP[0].y * ((int32_t)tchP[1].y * (int32_t)tchP[2].x - (int32_t)tchP[1].x * (int32_t)tchP[2].y)
            + (int32_t)tchP[0].x * ((int32_t)scrP[1].y * (int32_t)tchP[2].y - (int32_t)scrP[2].y * (int32_t)tchP[1].y)
            + (int32_t)tchP[0].y * ((int32_t)scrP[2].y * (int32_t)tchP[1].x - (int32_t)scrP[1].y * (int32_t)tchP[2].x)
        )<<shift) / divisor
    };

    // float temp1, temp2;
    // float cal_A = 0.0, cal_B = 0.0, cal_C = 0.0, cal_D = 0.0, cal_E = 0.0, cal_F = 0.0;

    // //A
    // temp1 = (scrP[0].x * (tchP[1].y - tchP[2].y)) + (scrP[1].x * (tchP[2].y - tchP[0].y)) + (scrP[2].x * (tchP[0].y - tchP[1].y));
    // temp2 = (tchP[0].x * (tchP[1].y - tchP[2].y)) + (tchP[1].x * (tchP[2].y - tchP[0].y)) + (tchP[2].x * (tchP[0].y - tchP[1].y));
    // cal_A = temp1 / temp2;

    // //B
    // temp1 = (cal_A * (tchP[2].x - tchP[1].x)) + scrP[1].x - scrP[2].x;
    // temp2 = tchP[1].y - tchP[2].y;
    // cal_B = temp1 / temp2;
    // // cali_B = (int32_t) ((double)cal_B * RESCALE_FACTOR);

    // //C
    // cal_C = scrP[2].x - (cal_A * tchP[2].x) - (cal_B * tchP[2].y);
    // // cali_C = (int32_t) (cal_C * RESCALE_FACTOR);

    // //D
    // temp1 = (scrP[0].y * (tchP[1].y - tchP[2].y)) + (scrP[1].y * (tchP[2].y - tchP[0].y)) + (scrP[2].y * (tchP[0].y - tchP[1].y));
    // temp2 = (tchP[0].x * (tchP[1].y - tchP[2].y)) + (tchP[1].x * (tchP[2].y - tchP[0].y)) + (tchP[2].x * (tchP[0].y - tchP[1].y));
    // cal_D = temp1 / temp2;
    // // cali_D = (int32_t) (cal_D * RESCALE_FACTOR);

    // //E
    // temp1 = (cal_D * (tchP[2].x - tchP[1].x)) + scrP[1].y - scrP[2].y;
    // temp2 = tchP[1].y - tchP[2].y;
    // cal_E = temp1 / temp2;
    // // cali_E = (int32_t) (cal_E * RESCALE_FACTOR);

    // //F
    // cal_F = scrP[2].y - cal_D * tchP[2].x - cal_E * tchP[2].y;

    // int32_t temp1, temp2;
    // int32_t cal_A = 0,
    //         cal_B = 0,
    //         cal_C = 0,
    //         cal_D = 0,
    //         cal_E = 0,
    //         cal_F = 0;

    // //A
    // temp1 = ((int32_t)scrP[0].x * ((int32_t)tchP[1].y - (int32_t)tchP[2].y)) + (scrP[1].x * ((int32_t)tchP[2].y - (int32_t)tchP[0].y)) + ((int32_t)scrP[2].x * ((int32_t)tchP[0].y - (int32_t)tchP[1].y));
    // temp2 = ((int32_t)tchP[0].x * ((int32_t)tchP[1].y - (int32_t)tchP[2].y)) + (tchP[1].x * ((int32_t)tchP[2].y - (int32_t)tchP[0].y)) + ((int32_t)tchP[2].x * ((int32_t)tchP[0].y - (int32_t)tchP[1].y));
    // cal_A = (temp1<<16) / temp2;

    // //B
    // temp1 = (cal_A * ((int32_t)tchP[2].x - (int32_t)tchP[1].x)) + (((int32_t)scrP[1].x - (int32_t)scrP[2].x)<<16);
    // temp2 = (int32_t)tchP[1].y - (int32_t) tchP[2].y;
    // cal_B = temp1 / temp2;

    // //C
    // cal_C = (((int32_t)scrP[2].x)<<16) - (cal_A * (int32_t)tchP[2].x) - (cal_B * (int32_t)tchP[2].y);

    // //D
    // temp1 = ((int32_t)scrP[0].y * ((int32_t)tchP[1].y - (int32_t)tchP[2].y)) + ((int32_t)scrP[1].y * ((int32_t)tchP[2].y - (int32_t)tchP[0].y)) + ((int32_t)scrP[2].y * ((int32_t)tchP[0].y - (int32_t)tchP[1].y));
    // temp2 = ((int32_t)tchP[0].x * ((int32_t)tchP[1].y - (int32_t)tchP[2].y)) + ((int32_t)tchP[1].x * ((int32_t)tchP[2].y - (int32_t)tchP[0].y)) + ((int32_t)tchP[2].x * ((int32_t)tchP[0].y - (int32_t)tchP[1].y));
    // cal_D = (temp1<<16) / temp2;

    // //E
    // temp1 = (cal_D * ((int32_t)tchP[2].x - (int32_t)tchP[1].x)) + (((int32_t)scrP[1].y - (int32_t)scrP[2].y)<<16);
    // temp2 = (int32_t)tchP[1].y - (int32_t)tchP[2].y;
    // cal_E = temp1 / temp2;

    // //F
    // cal_F = (((int32_t) scrP[2].y)<<16) - (cal_D * (int32_t) tchP[2].x) - (cal_E * (int32_t) tchP[2].y);

    // lv_tc_coeff_t result = {
    //     true,
    //     cal_A,
    //     cal_B,
    //     cal_C,
    //     cal_D,
    //     cal_E,
    //     cal_F
    // };

    lv_tc_set_coeff(result, save);

    #ifdef ESP_PLATFORM
        ESP_LOGI(TAG, "touch calibration coefficients -> [a: %f, b: %f, c: %f, d: %f, e: %f, f: %f]", result.a, result.b,
                result.c, result.d, result.e, result.f);
    #endif
}

lv_point_t _lv_tc_transform_point_indev(lv_indev_data_t *data) {
    if(data->state == LV_INDEV_STATE_PRESSED) {
        return lv_tc_transform_point(data->point);
    } else {
        //Reject invalid points if the touch panel is in released state
        //lv_point_t point = {0, 0};
        return data->point;
    }
}

lv_point_t lv_tc_transform_point(lv_point_t point) {
    lv_point_t transformedPoint = point;
    if(calibResult.isValid) {
        // transformedPoint.x = roundf((lv_tc_val_t)point.x * calibResult.a + (lv_tc_val_t)point.y * calibResult.b + calibResult.c);
        // transformedPoint.y = roundf((lv_tc_val_t)point.x * calibResult.d + (lv_tc_val_t)point.y * calibResult.e + calibResult.f);

        // transformedPoint.x = (calibResult.a * point.x + calibResult.b * point.y + calibResult.c);// / RESCALE_FACTOR;
        // transformedPoint.y = (calibResult.d * point.x + calibResult.e * point.y + calibResult.f);// / RESCALE_FACTOR;

        transformedPoint.x = (int32_t)((int32_t)calibResult.a * (int32_t)point.x + (int32_t)calibResult.b * (int32_t)point.y + (int32_t)calibResult.c)>>8;
        transformedPoint.y = (int32_t)((int32_t)calibResult.d * (int32_t)point.x + (int32_t)calibResult.e * (int32_t)point.y + (int32_t)calibResult.f)>>8;
    }

    return transformedPoint;
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_tc_indev_drv_read_cb(lv_indev_drv_t *indevDrv, lv_indev_data_t *data) {
    if(!indevDrv->user_data) return;

    //Call the actual indev read callback
    ((void (*)(lv_indev_drv_t*, lv_indev_data_t*))indevDrv->user_data)(indevDrv, data);

    //Pass the results to an ongoing calibration if there is one
    if(registeredTCScreen && registeredInputCb && registeredTCScreen == lv_scr_act()) {
        if(!registeredInputCb(registeredTCScreen, data)) {
            //Override state and point if the input has been handled by the registered calibration screen
            data->state = LV_INDEV_STATE_RELEASED;
            lv_point_t point = {0, 0};
            data->point = point;

            return;
        }
    }

    data->point = _lv_tc_transform_point_indev(data);
}