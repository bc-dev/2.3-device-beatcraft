#include <hardware/gps.h> 

#define LOG_TAG "bc10-gps"
#include <utils/Log.h>

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <semaphore.h>

//  logging macro.
#define BC10_GPS_DEBUG(...) LOG(LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define BC10_GPS_ERROR(...) LOG(LOG_ERROR, LOG_TAG, __VA_ARGS__)

//  GPS serial interface read point.
//  TODO: dynamically set device filename
#define TTY_DEV "/dev/ttyS1"

//  GPS status setting macro
#define setGpsStatus(_cb, _s)    \
  if ((_cb).status_cb) {          \
    GpsStatus gps_status;         \
    gps_status.status = (_s);     \
    (_cb).status_cb(&gps_status); \
    BC10_GPS_DEBUG("gps status callback: 0x%x", _s); \
  }

//  NMEA parser locking macro.
#define GPS_STATE_LOCK_FIX(_s)         \
{                                      \
  int ret;                             \
  do {                                 \
    ret = sem_wait(&(_s)->fix_sem);    \
  } while (ret < 0 && errno == EINTR);   \
}

#define GPS_STATE_UNLOCK_FIX(_s)       \
  sem_post(&(_s)->fix_sem)

//
//  Nmea Parser stuff
//
#define  NMEA_MAX_SIZE  83

enum {
    STATE_QUIT  = 0,
    STATE_INIT  = 1,
    STATE_START = 2
};

typedef struct {
    int            pos;
    int            overflow;
    int            utc_year;
    int            utc_mon;
    int            utc_day;
    int            utc_diff;
    GpsLocation    fix;
    GpsSvStatus    sv_status;
    int            sv_status_changed;
    char           in[ NMEA_MAX_SIZE+1 ];
} NmeaReader;

//  
//  GPSState Structure
//
typedef struct {
    int             init;
    int             fd;
    FILE            *fp;
    GpsCallbacks    callbacks;
    pthread_t       thread;
    int             fix_freq;
    sem_t           fix_sem;
    int             first_fix;
} bc10_GpsState;

static bc10_GpsState _gps_state[1];
static bc10_GpsState *gps_state = _gps_state;


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   T O K E N I Z E R                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  32

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int
nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
    int    count = 0;
    char*  q;

    // the initial '$' is optional
    if (p < end && p[0] == '$')
        p += 1;

    // remove trailing newline
    if (end > p && end[-1] == '\n') {
        end -= 1;
        if (end > p && end[-1] == '\r')
            end -= 1;
    }

    // get rid of checksum at the end of the sentecne
    if (end >= p+3 && end[-3] == '*') {
        end -= 3;
    }

    while (p < end) {
        const char*  q = p;

        q = memchr(p, ',', end-p);
        if (q == NULL)
            q = end;

        if (count < MAX_NMEA_TOKENS) {
            t->tokens[count].p   = p;
            t->tokens[count].end = q;
            count += 1;
        }

        if (q < end)
            q += 1;

        p = q;
    }

    t->count = count;
    return count;
}

static Token
nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";

    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    } else
        tok = t->tokens[index];

    return tok;
}


static int
str2int( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;

    if (len == 0) {
      return -1;
    }

    for ( ; len > 0; len--, p++ )
    {
        int  c;

        if (p >= end)
            goto Fail;

        c = *p - '0';
        if ((unsigned)c >= 10)
            goto Fail;

        result = result*10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double
str2float( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;
    char  temp[16];

    if (len == 0) {
      return -1.0;
    }

    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy( temp, p, len );
    temp[len] = 0;
    return strtod( temp, NULL );
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static void
nmea_reader_update_utc_diff( NmeaReader*  r )
{
    time_t         now = time(NULL);
    struct tm      tm_local;
    struct tm      tm_utc;
    long           time_local, time_utc;

    gmtime_r( &now, &tm_utc );
    localtime_r( &now, &tm_local );

    time_local = tm_local.tm_sec +
                 60*(tm_local.tm_min +
                 60*(tm_local.tm_hour +
                 24*(tm_local.tm_yday +
                 365*tm_local.tm_year)));

    time_utc = tm_utc.tm_sec +
               60*(tm_utc.tm_min +
               60*(tm_utc.tm_hour +
               24*(tm_utc.tm_yday +
               365*tm_utc.tm_year)));

    r->utc_diff = time_utc - time_local;
}


static void
nmea_reader_init( NmeaReader*  r )
{
    memset( r, 0, sizeof(*r) );

    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;
//    r->callback = NULL;
    r->fix.size = sizeof( r->fix );

    nmea_reader_update_utc_diff( r );
}

/*
static void
nmea_reader_set_callback( NmeaReader*  r, gps_location_callback  cb )
{
    r->callback = cb;
    if (cb != NULL && r->fix.flags != 0) {
        D("%s: sending latest fix to new callback", __FUNCTION__);
        r->callback( &r->fix );
        r->fix.flags = 0;
    }
}
*/

static int
nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    int        hour, minute;
    double     seconds;
    struct tm  tm;
    time_t     fix_time;

    if (tok.p + 6 > tok.end)
        return -1;

    if (r->utc_year < 0) {
        // no date yet, get current one
        time_t  now = time(NULL);
        gmtime_r( &now, &tm );
        r->utc_year = tm.tm_year + 1900;
        r->utc_mon  = tm.tm_mon + 1;
        r->utc_day  = tm.tm_mday;
    }

    hour    = str2int(tok.p,   tok.p+2);
    minute  = str2int(tok.p+2, tok.p+4);
    seconds = str2float(tok.p+4, tok.end);

    tm.tm_hour = hour;
    tm.tm_min  = minute;
    tm.tm_sec  = (int) seconds;
    tm.tm_year = r->utc_year - 1900;
    tm.tm_mon  = r->utc_mon - 1;
    tm.tm_mday = r->utc_day;

    fix_time = mktime( &tm ) + r->utc_diff;
    r->fix.timestamp = (long long)fix_time * 1000;
    return 0;
}

static int
nmea_reader_update_cdate( NmeaReader*  r, 
                          Token  tok_d, Token  tok_m, Token  tok_y )
{

    if ( (tok_d.p + 2 > tok_d.end) ||
         (tok_m.p + 2 > tok_m.end) ||
         (tok_y.p + 4 > tok_y.end) )
        return -1;

    r->utc_day  = str2int(tok_d.p, tok_d.p+2);
    r->utc_mon  = str2int(tok_m.p, tok_m.p+2);
    r->utc_year = str2int(tok_y.p, tok_y.end+4);

    return 0;
}

static int
nmea_reader_update_date( NmeaReader*  r, Token  date, Token  time )
{
    Token  tok = date;
    int    day, mon, year;

    if (tok.p + 6 != tok.end) {
        BC10_GPS_DEBUG("date not properly formatted: '%.*s'", 
                       tok.end-tok.p, tok.p);
        return -1;
    }
    day  = str2int(tok.p, tok.p+2);
    mon  = str2int(tok.p+2, tok.p+4);
    year = str2int(tok.p+4, tok.p+6) + 2000;

    if ((day|mon|year) < 0) {
        BC10_GPS_DEBUG("date not properly formatted: '%.*s'", 
                       tok.end-tok.p, tok.p);
        return -1;
    }

    r->utc_year  = year;
    r->utc_mon   = mon;
    r->utc_day   = day;

    return nmea_reader_update_time( r, time );
}

static double
convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}

static int
nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
    double   lat, lon;
    Token    tok;

    tok = latitude;
    if (tok.p + 6 > tok.end) {
        BC10_GPS_DEBUG("latitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lat = convert_from_hhmm(tok);
    if (latitudeHemi == 'S')
        lat = -lat;

    tok = longitude;
    if (tok.p + 6 > tok.end) {
        BC10_GPS_DEBUG("longitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lon = convert_from_hhmm(tok);
    if (longitudeHemi == 'W')
        lon = -lon;

    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int
nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    double  alt;
    Token   tok = altitude;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
    return 0;
}

static int
nmea_reader_update_accuracy( NmeaReader*  r,
                             Token        accuracy )
{
    double  acc;
    Token   tok = accuracy;

    if (tok.p >= tok.end)
        return -1;

    r->fix.accuracy = str2float(tok.p, tok.end);

    if (r->fix.accuracy == 99.99) {
      return 0;
    }

    r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY;
    return 0;
}

static int
nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
    double  alt;
    Token   tok = bearing;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
    return 0;
}


static int
nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    double  alt;
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
    r->fix.speed    = str2float(tok.p, tok.end);
    return 0;
}


static void
nmea_reader_parse( NmeaReader*  r )
{
    /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;

    BC10_GPS_DEBUG("Received: '%.*s'", r->pos, r->in);
    if (r->pos < 9) {
//        D("Too short. discarded.");
        return;
    }

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
#if GPS_DEBUG
    {
        int  n;
        BC10_GPS_DEBUG("Found %d tokens", tzer->count);
        for (n = 0; n < tzer->count; n++) {
            Token  tok = nmea_tokenizer_get(tzer,n);
            BC10_GPS_DEBUG("%2d: '%.*s'", n, tok.end-tok.p, tok.p);
        }
    }
#endif

    tok = nmea_tokenizer_get(tzer, 0);
    if (tok.p + 5 > tok.end) {
        BC10_GPS_DEBUG("sentence id '%.*s' too short, ignored.", 
                       tok.end-tok.p, tok.p);
        return;
    }

    // ignore first two characters.
    tok.p += 2;
    if ( !memcmp(tok.p, "GGA", 3) ) {
        // GPS fix
        Token  tok_fixstaus = nmea_tokenizer_get(tzer, 6);

        if (tok_fixstaus.p[0] > '0') {
            Token  tok_time          = nmea_tokenizer_get(tzer, 1);
            Token  tok_latitude      = nmea_tokenizer_get(tzer, 2);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer, 3);
            Token  tok_longitude     = nmea_tokenizer_get(tzer, 4);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer, 5);
            Token  tok_altitude      = nmea_tokenizer_get(tzer, 9);
            Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

            nmea_reader_update_time( r, tok_time );
            nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );
            nmea_reader_update_altitude( r, tok_altitude, tok_altitudeUnits );
        }

    } else if ( !memcmp(tok.p, "GLL", 3) ) {
        Token  tok_fixstaus = nmea_tokenizer_get(tzer, 6);

        if (tok_fixstaus.p[0] == 'A') {
            Token  tok_latitude      = nmea_tokenizer_get(tzer, 1);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer, 2);
            Token  tok_longitude     = nmea_tokenizer_get(tzer, 3);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer, 4);
            Token  tok_time          = nmea_tokenizer_get(tzer, 5);

            nmea_reader_update_time( r, tok_time );
            nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );
        }

    } else if ( !memcmp(tok.p, "GSA", 3) ) {
        Token  tok_fixStatus = nmea_tokenizer_get(tzer, 2);
        int i;

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1') {
            Token  tok_accuracy = nmea_tokenizer_get(tzer,15);

            nmea_reader_update_accuracy( r, tok_accuracy );

            r->sv_status.used_in_fix_mask = 0ul;

            for (i = 3; i <= 14; ++i) {
                Token  tok_prn  = nmea_tokenizer_get(tzer, i);
                int prn = str2int(tok_prn.p, tok_prn.end);

                if (prn > 0) {
                    r->sv_status.used_in_fix_mask |= (1ul << (32 - prn));
                    r->sv_status_changed = 1;
                    BC10_GPS_DEBUG("%s: fix mask is %d", __FUNCTION__, 
                                   r->sv_status.used_in_fix_mask);
                }

            }

        }

    } else if ( !memcmp(tok.p, "GSV", 3) ) {
        Token  tok_noSatellites  = nmea_tokenizer_get(tzer, 3);
        int noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);
       
        if (noSatellites > 0) {
            Token  tok_noSentences = nmea_tokenizer_get(tzer, 1);
            Token  tok_sentence    = nmea_tokenizer_get(tzer, 2);

            int sentence = str2int(tok_sentence.p, tok_sentence.end);
            int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
            int curr;
            int i;
          
            if (sentence == 1) {
                r->sv_status_changed = 0;
                r->sv_status.num_svs = 0;
            }

            curr = r->sv_status.num_svs;

            i = 0;

            while (i < 4 && r->sv_status.num_svs < noSatellites) {
                Token  tok_prn       = nmea_tokenizer_get(tzer, i * 4 + 4);
                Token  tok_elevation = nmea_tokenizer_get(tzer, i * 4 + 5);
                Token  tok_azimuth   = nmea_tokenizer_get(tzer, i * 4 + 6);
                Token  tok_snr       = nmea_tokenizer_get(tzer, i * 4 + 7);

                r->sv_status.sv_list[curr].prn
                    = str2int(tok_prn.p, tok_prn.end);
                r->sv_status.sv_list[curr].elevation
                    = str2float(tok_elevation.p, tok_elevation.end);
                r->sv_status.sv_list[curr].azimuth
                    = str2float(tok_azimuth.p, tok_azimuth.end);
                r->sv_status.sv_list[curr].snr
                    = str2float(tok_snr.p, tok_snr.end);

                r->sv_status.num_svs += 1;

                curr += 1;

                i += 1;
          }

          if (sentence == totalSentences) {
              r->sv_status_changed = 1;
          }
          BC10_GPS_DEBUG("%s: GSV message with total satellites %d", 
                         __FUNCTION__, noSatellites);   
        }

    } else if ( !memcmp(tok.p, "RMC", 3) ) {
        Token  tok_fixStatus = nmea_tokenizer_get(tzer, 2);

        if (tok_fixStatus.p[0] == 'A') {
            Token  tok_time          = nmea_tokenizer_get(tzer, 1);
            Token  tok_latitude      = nmea_tokenizer_get(tzer, 3);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer, 4);
            Token  tok_longitude     = nmea_tokenizer_get(tzer, 5);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer, 6);
            Token  tok_speed         = nmea_tokenizer_get(tzer, 7);
            Token  tok_bearing       = nmea_tokenizer_get(tzer, 8);
            Token  tok_date          = nmea_tokenizer_get(tzer, 9);

            nmea_reader_update_date( r, tok_date, tok_time );

            nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed( r, tok_speed );
        }

    } else if ( !memcmp(tok.p, "VTG", 3) ) {
        Token  tok_fixStatus = nmea_tokenizer_get(tzer, 9);

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != 'N') {
            Token  tok_bearing = nmea_tokenizer_get(tzer, 1);
            Token  tok_speed   = nmea_tokenizer_get(tzer, 5);

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }

    } else if ( !memcmp(tok.p, "ZDA", 3) ) {
        Token  tok_time;
        Token  tok_year = nmea_tokenizer_get(tzer, 4);

        if (tok_year.p[0] != '\0') {
          Token  tok_day = nmea_tokenizer_get(tzer, 2);
          Token  tok_mon = nmea_tokenizer_get(tzer, 3);

          nmea_reader_update_cdate( r, tok_day, tok_mon, tok_year );
        }

        tok_time  = nmea_tokenizer_get(tzer, 1);

        if (tok_time.p[0] != '\0') {

          nmea_reader_update_time( r, tok_time );

        }


    } else {
        tok.p -= 2;
        BC10_GPS_DEBUG("unknown sentence '%.*s", tok.end-tok.p, tok.p);
    }

    if (!gps_state->first_fix &&
        r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {

        if (gps_state->callbacks.location_cb) {
            gps_state->callbacks.location_cb( &r->fix );
            r->fix.flags = 0;
        }

        gps_state->first_fix = 1;
    }
}

static void
nmea_reader_addc( NmeaReader*  r, int  c )
{
    if (r->overflow) {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }

    r->in[r->pos] = (char)c;
    r->pos       += 1;

    if (c == '\n') {
        GPS_STATE_LOCK_FIX(gps_state);
        nmea_reader_parse( r );
        GPS_STATE_UNLOCK_FIX(gps_state);
        r->pos = 0;
    }
}

/**                                        */
/** standard GPS interface implementation  */
/**                                        */

const void* bc10_gps_get_extention(const char* name)
{
    BC10_GPS_DEBUG("bc10_gps_get_extension called!:%s",name);

    //
    //   bc10 does not have NO EXTENSION.
    //

    return 0;
}

void bc10_gps_delete_aiding_data(GpsAidingData flags)
{
    BC10_GPS_DEBUG("bc10_gps_delete_aiding_data called!");

    //
    //   This method is invoked via GpsLocationProvider#sendExtraCommand.
    //   We usually get Aiding data via mobile phone network and it will
    //   be used to assist position calculation in unfavorable situation.
    //
    //   At the moment, We does not use aiding data.
    //   @see http://en.wikipedia.org/wiki/Assisted_GPS
    //

    return;
}

int bc10_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    BC10_GPS_DEBUG("bc10_gps_inject_time called!");

    //
    //   We don't have to implement this method, because 
    //   does not use GPS via network.
    //
    //   @see http://en.wikipedia.org/wiki/Assisted_GPS
    //

    return 0;
}

void bc10_gps_set_fix_frequency(int frequency)
{
    BC10_GPS_DEBUG("bc10_gps_set_fix_frequency called!");

    GPS_STATE_LOCK_FIX(gps_state);
    gps_state->fix_freq = frequency;
    GPS_STATE_UNLOCK_FIX(gps_state);
    
    return;
}

int bc10_gps_set_position_mode(GpsPositionMode mode, int fix_frequency)
{
    //   This method is called via bc10_gps_start function.
    BC10_GPS_DEBUG("bc10_gps_set_position_mode called!");

    //
    //   Gps Position mode is fixed value(GPS_POSITION_MODE_STANDALONE)
    //   for us.
    //
    if (mode != GPS_POSITION_MODE_STANDALONE) {
        BC10_GPS_ERROR("Invalid Gps Position mode!: %d", mode);
        return -1;
    }

    bc10_gps_set_fix_frequency(fix_frequency);

    return 0;
}

void bc10_gps_cleanup(void)
{
    BC10_GPS_DEBUG("bc10_gps_set_fix_cleanup called!");

    //  cleanup
    setGpsStatus(gps_state->callbacks, GPS_STATUS_ENGINE_OFF);
    close(gps_state->fd);
    fclose(gps_state->fp);
    
    return;
}

int bc10_gps_stop(void)
{
    BC10_GPS_DEBUG("bc10_gps_stop called!");

    //    gps reader thread automatically stops.
    gps_state->init = STATE_QUIT;
    pthread_join(gps_state->thread, NULL);

    setGpsStatus(gps_state->callbacks, GPS_STATUS_SESSION_END);
    return 0;
}

static void* 
bc10_gps_reader_thread(void *args)
{
    BC10_GPS_DEBUG("bc10_gps_reader_thread started!");

    char buf[512];
    NmeaReader reader;
    int len, nn, ret;

    //  set init value
    ret = 0;
    ret = fputs("$PSRF104,35,139,0,96000,407952,1557,12,1*29\r\n", gps_state->fp);
    //ret = fputs("$PSRF104,0,0,0,0,0,0,12,1*10\r\n", gps_state->fp);
    if (ret == EOF) {
        BC10_GPS_ERROR("bc10_gps_reader_thread: init value set error!(104)");
    } else {
        BC10_GPS_DEBUG("wrote initial string -> $PSRF104,35,139,0,96000,407952,1557,12,1*29");
    }

    ret = fputs("$PSRF106,21*0F\r\n", gps_state->fp);
    if (ret == EOF) {
        BC10_GPS_ERROR("bc10_gps_reader_thread: init value set error!(106)");
    } else {
        BC10_GPS_DEBUG("wrote initial string -> $PSRF106,21*0F");
    }
    nmea_reader_init( &reader );
    do {
        if (fgets(buf, sizeof(buf), gps_state->fp) != NULL) {
            len = strlen(buf);
            for (nn = 0; nn < len; nn++) {
                nmea_reader_addc( &reader, buf[nn] );
            } 
        }
	sleep(gps_state->fix_freq);
    } while (gps_state->init == STATE_START);

    BC10_GPS_DEBUG("bc10_gps_reader_thread ended!");

    return 0;
}

int bc10_gps_start(void)
{
    BC10_GPS_DEBUG("bc10_gps_start called!");

    int ret;
    setGpsStatus(gps_state->callbacks, GPS_STATUS_SESSION_BEGIN);

    //
    //  start gps reader thread
    //
    ret = pthread_create(
        &gps_state->thread,
        NULL,
        bc10_gps_reader_thread,
        NULL
    );
    if (ret != 0) {    
        BC10_GPS_ERROR("bc10_gps_start failed because of thread creation failure: %d", ret);
        return ret;
    }
    gps_state->init = STATE_START;

    return 0;
}

static int bc10_gps_term_init(int fd)
{
    int ret;
    unsigned int speed;

    struct termios ios;
    memset(&ios, 0x0, sizeof(ios));
    ret = tcgetattr(fd, &ios);
    if (ret < 0) {
        BC10_GPS_ERROR("bc10_gps_term_init: serial port attribute get failed!");
        return 1;
    }
    
    speed = cfgetispeed(&ios);
    BC10_GPS_DEBUG("bc10_gps_term_init: got serial port speed %u", speed);

    ret = cfsetispeed(&ios, B57600);
    if (ret < 0) {
        BC10_GPS_ERROR("bc10_gps_term_init: serial port setspeed failed!");
        return 1;
    }

    speed = cfgetispeed(&ios);
    BC10_GPS_DEBUG("bc10_gps_term_init: set serial port speed %u", speed);

    if (ios.c_cflag | CRTSCTS) {
        BC10_GPS_DEBUG("bc10_gps_term_init: hardware flow control is enabled");
        ios.c_cflag &= ~CRTSCTS;
        BC10_GPS_DEBUG("bc10_gps_term_init: disable hardware flow control");
    }

    ret = tcsetattr(fd, TCSANOW, &ios);
    if (ret < 0) {
        BC10_GPS_ERROR("bc10_gps_term_init: serial port attribute set failed!");
        return 1;
    }
    
    BC10_GPS_DEBUG("bc10_gps_term_init: success %s", TTY_DEV);

    return 0;
}


int bc10_gps_init(GpsCallbacks *callbacks)
{
    BC10_GPS_DEBUG("bc10_gps_init called!");

    gps_state->callbacks = *callbacks;
    setGpsStatus(gps_state->callbacks, GPS_STATUS_NONE);

    FILE* fp;
    int fd = open(TTY_DEV, O_RDWR);
    int ret = 0;
    if (fd < 0) {
        BC10_GPS_ERROR("bc10_gps_init: gps device open failed! : %s", TTY_DEV);
        return 1;
    }

    BC10_GPS_DEBUG("bc10_gps_init: successfully opened %s", TTY_DEV);

    ret = sem_init(&gps_state->fix_sem , 0, 1);
    if (ret == -1) {
        BC10_GPS_ERROR("bc10_gps_init: gps fix_sem init failed!");
        return 1;
    }

    ret = bc10_gps_term_init(fd);
    if (ret != 0) {
        BC10_GPS_ERROR("bc10_gps_init: gps device init failed!");
        return 1;
    }
    
    setGpsStatus(gps_state->callbacks, GPS_STATUS_ENGINE_ON);

    fp = fdopen(fd, "w+");
    if (!fp) {
        BC10_GPS_ERROR("bc10_gps_init: device file open(fdopen) failed!");
        return 1;
    }

    gps_state->init = STATE_INIT;
    gps_state->fd = fd;
    gps_state->fp = fp;

    BC10_GPS_DEBUG("bc10_gps_init: success");

    return 0;
}


//
//  Represents the standard GPS interface.
//
static const GpsInterface bc10GpsInterface = {
    sizeof(GpsInterface),
    bc10_gps_init,
    bc10_gps_start,
    bc10_gps_stop,
    bc10_gps_set_fix_frequency,
    bc10_gps_cleanup,
    bc10_gps_inject_time,
    bc10_gps_delete_aiding_data,
    bc10_gps_set_position_mode,
    bc10_gps_get_extention,
};

const GpsInterface* gps__get_gps_interface(struct gps_device_t* dev)
{
    BC10_GPS_DEBUG("gps_get_hardware_interface called!");
    return &bc10GpsInterface;
}

static int open_gps(const struct hw_module_t* module, char const* name,
        struct hw_device_t** device)
{
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
//    dev->common.close = (int (*)(struct hw_device_t*))close_lights;
    dev->get_gps_interface = gps__get_gps_interface;

    *device = (struct hw_device_t*)dev;
    return 0;
}


static struct hw_module_methods_t gps_module_methods = {
    .open = open_gps
};

const struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "bc10 GPS Module",
    .author = "BeatCraft, Inc.",
    .methods = &gps_module_methods,
};

