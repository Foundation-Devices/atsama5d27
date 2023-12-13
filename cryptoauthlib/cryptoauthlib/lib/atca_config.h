/* Auto-generated config file atca_config.h */
#ifndef ATCA_CONFIG_H
#define ATCA_CONFIG_H

/* Included HALS */
/* #undef ATCA_HAL_KIT_UART */
/* #undef ATCA_HAL_KIT_HID */
/* #undef ATCA_HAL_I2C */
/* #undef ATCA_HAL_SPI */
/* #undef ATCA_HAL_KIT_BRIDGE */
/* #undef ATCA_HAL_CUSTOM */
#define ATCA_HAL_SWI_UART
/* #undef ATCA_HAL_1WIRE */

/* Included device support */
/* #undef ATCA_ATSHA204A_SUPPORT */
/* #undef ATCA_ATSHA206A_SUPPORT */
/* #undef ATCA_ATECC108A_SUPPORT */
/* #undef ATCA_ATECC508A_SUPPORT */
#define ATCA_ATECC608_SUPPORT
/* #undef ATCA_ECC204_SUPPORT */
/* #undef ATCA_TA010_SUPPORT */
/* #undef ATCA_SHA104_SUPPORT */
/* #undef ATCA_SHA105_SUPPORT */

/* Linked device support library */
#define ATCA_TA_SUPPORT 0

/** Device Override - Library Assumes ATECC608B support in checks */
/* #undef ATCA_ATECC608A_SUPPORT */

/** Define to enable compatibility with legacy HALs
   (HALs with embedded device logic)*/
/* #undef ATCA_HAL_LEGACY_API */

/** To use dynamically registered HALs without any of the provided
implementations its necessary to specify a value here - using this
in addition to specifying a provide hal may result in compilation
problems - it will need to be the same as the number of the hal options
selected plus however additional slots one would like */
/* #undef ATCA_MAX_HAL_CACHE */

/** Define if cryptoauthlib is to use the maximum execution time method */
/* #undef ATCA_NO_POLL */

/* \brief How long to wait after an initial wake failure for the POST to
 *         complete.
 * If Power-on self test (POST) is enabled, the self test will run on waking
 * from sleep or during power-on, which delays the wake reply.
 */
#ifndef ATCA_POST_DELAY_MSEC
#define ATCA_POST_DELAY_MSEC 25
#endif

/***************** Diagnostic & Test Configuration Section *****************/

/** Enable debug messages */
/* #undef ATCA_PRINTF */

/** Enable preprocessor warning messages */
#define ATCA_PREPROCESSOR_WARNING 0

/** Enable jwt functionality */
/* #undef ATCA_JWT_EN */

/** Enable to build in test hooks */
/* #undef ATCA_TESTS_ENABLED */

/******************** Features Configuration Section ***********************/

/** Define certificate templates to be supported. */
/* #undef ATCA_TNGTLS_SUPPORT */
/* #undef ATCA_TNGLORA_SUPPORT */
/* #undef ATCA_TFLEX_SUPPORT */
/* #undef ATCA_TNG_LEGACY_SUPPORT */

/** Define Software Crypto Library to Use - if none are defined use the
    cryptoauthlib version where applicable */
/* #undef ATCA_MBEDTLS */
/* #undef ATCA_OPENSSL */
/* #undef ATCA_WOLFSSL */

#ifdef ATCA_WOLFSSL
/* #undef WOLFSSL_USER_SETTINGS */
#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#endif
#endif

/** Additional Runtime Configuration */
/* #undef ATCA_LIBRARY_CONF */

/** Define to build atcab_ functions rather that defining them as macros */
/* #undef ATCA_USE_ATCAB_FUNCTIONS */

/** Define to enable older API forms that have been replaced */
/* #undef ATCA_ENABLE_DEPRECATED */

/** Enable Strict ISO/C99 compliance */
/* #undef ATCA_STRICT_C99 */

/******************** Device Configuration Section *************************/

/** Enable the delete command */
#define CALIB_DELETE_EN 0

/******************** Packet Size Configuration Section *************************/

/** Provide Maximum packet size for the command to be sent and received */
#ifndef MAX_PACKET_SIZE
#define MAX_PACKET_SIZE         (1072U)
#endif

#define MULTIPART_BUF_EN 0

/******************** Platform Configuration Section ***********************/

/** Define if the library is not to use malloc/free */
#define ATCA_NO_HEAP

/** Define platform provided functions */
/* #undef ATCA_PLATFORM_MALLOC */
/* #undef ATCA_PLATFORM_FREE */
/* #undef ATCA_PLATFORM_STRCASESTR */
/* #undef ATCA_PLATFORM_MEMSET_S */

#define atca_delay_ms   hal_delay_ms
#define atca_delay_us   hal_delay_us

#endif // ATCA_CONFIG_H
