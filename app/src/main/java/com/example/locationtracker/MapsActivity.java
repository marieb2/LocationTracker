package com.example.locationtracker;


import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.support.v4.app.ActivityCompat;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.util.Log;

import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.Circle;
import com.google.android.gms.maps.model.CircleOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.tasks.OnSuccessListener;

import java.util.ArrayList;
import java.util.List;

public class MapsActivity extends FragmentActivity implements OnMapReadyCallback, LocationListener, SensorEventListener {

    private GoogleMap mMap;

    LocationManager locationManager;

    private Sensor accelSensor;

    private SensorManager mySensorManager;

    private LatLng currentLocation ;

    private Location lastLocation ;

    private FusedLocationProviderClient fusedLocationClient;

    AverageDistance myChunk = new AverageDistance();

    boolean isUpdating = false;

    private Long tsLong ;

    CircleOptions circleOptions = new CircleOptions();
    Circle mapCircle;

    private int accelCount = 0;

    float bearing = 0.0f;



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_maps);
        // Obtain the SupportMapFragment and get notified when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);

        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);

        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 30000, 0, (LocationListener) this);

        fusedLocationClient = LocationServices.getFusedLocationProviderClient(this);

        mySensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);

        accelSensor = mySensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

    }

    @Override
    protected void onResume() {
        super.onResume();
        mySensorManager.registerListener(this, accelSensor, mySensorManager.SENSOR_DELAY_NORMAL);
    }

    /**
     * Manipulates the map once available.
     * This callback is triggered when the map is ready to be used.
     * This is where we can add markers or lines, add listeners or move the camera. In this case,
     * we just add a marker near Sydney, Australia.
     * If Google Play services is not installed on the device, the user will be prompted to install
     * it inside the SupportMapFragment. This method will only be triggered once the user has
     * installed Google Play services and returned to the app.
     */
    @Override
    public void onMapReady(GoogleMap googleMap) {
        mMap = googleMap;

        // Add a marker in Sydney and move the camera
        //LatLng sydney = new LatLng(-34, 151);
        //mMap.addMarker(new MarkerOptions().position(sydney).title("Marker in Sydney"));
        //mMap.moveCamera(CameraUpdateFactory.newLatLng(sydney));

        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }

        fusedLocationClient.getLastLocation().addOnSuccessListener(this, new OnSuccessListener<Location>() {
                    @Override
                    public void onSuccess(Location location) {
                        // Got last known location. In some rare situations this can be null.
                        if (location != null) {
                            // Logic to handle location object
                            mMap.setMinZoomPreference(18);
                            LatLng newLocation ;
                            newLocation = new LatLng(location.getLatitude(),location.getLongitude());

                            bearing = location.getBearing();

                            currentLocation = newLocation ;

                            lastLocation = location ;

                            circleOptions.center(newLocation);

                            circleOptions.radius(6);
                            circleOptions.fillColor(Color.BLUE);
                            circleOptions.strokeWidth(6);

                            mapCircle = mMap.addCircle(circleOptions);

                            //mMap.addMarker(new MarkerOptions().position(newLocation).title("Current Location"));
                            mMap.moveCamera(CameraUpdateFactory.newLatLng(newLocation));
                        }else{
                            Log.d("NullLocation", "The last known location is NULL");
                        }
                    }
                });

    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        float[] mGravity = new float[3] ;
        float[] mMagnetic = new float[3] ;



            Log.i("MyActivity", "first if statement in onSensor Changed " );

            /*
            if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {

                mGravity = event.values.clone();

            }

            if(event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){

                mMagnetic = event.values.clone();

            }

            if (mGravity != null && mMagnetic != null) {

                Log.i("MyActivity", "updating bearing " );
                float[] rotationMatrix = new float[9];
                if (SensorManager.getRotationMatrix(rotationMatrix, null,
                        mGravity, mMagnetic)) {

                    float[] remappedRotationMatrix = new float[9];
                    switch (getWindowManager().getDefaultDisplay()
                            .getRotation()) {
                        case Surface.ROTATION_0:
                            SensorManager.remapCoordinateSystem(rotationMatrix,
                                    SensorManager.AXIS_X, SensorManager.AXIS_Y,
                                    remappedRotationMatrix);
                            break;
                        case Surface.ROTATION_90:
                            SensorManager.remapCoordinateSystem(rotationMatrix,
                            SensorManager.AXIS_Y,
                                    SensorManager.AXIS_MINUS_X,
                                    remappedRotationMatrix);
                            break;
                        case Surface.ROTATION_180:
                            SensorManager.remapCoordinateSystem(rotationMatrix,
                                    SensorManager.AXIS_MINUS_X,
                                    SensorManager.AXIS_MINUS_Y,
                                    remappedRotationMatrix);
                            break;
                        case Surface.ROTATION_270:
                            SensorManager.remapCoordinateSystem(rotationMatrix,
                                    SensorManager.AXIS_MINUS_Y,
                                    SensorManager.AXIS_X, remappedRotationMatrix);
                            break;
                    }

                    float results[] = new float[3];
                    SensorManager.getOrientation(remappedRotationMatrix,
                            results);

                   // bearing = results[0];


                    // Get measured value
                    float current_measured_bearing = (float) (results[0] * 180 / Math.PI);
                    if (current_measured_bearing < 0) {
                        current_measured_bearing += 360;
                    }
                    bearing = current_measured_bearing ;

                    Log.i("MyActivity", "first if statement in onSensor Changed bearing: " + bearing);
                }
            }*/


            if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){

                if(bearing != 0.0f){

                    Log.i("MyActivity", "in accelerometer = after bearing check " );
                    //update accelerometer data in myChunk
                    //use formula to convert distance to lat/lng
                    myChunk.xs.add(event.values[0]);
                    tsLong = System.currentTimeMillis();
                    myChunk.times.add(tsLong);
                    accelCount += 1 ;

                    if (accelCount == 7){
                        //get latLng from distance
                        //update location marker

                        double radius = 6371e3 ;

                        Float[] results ;
                        results = myChunk.getAverageDistance();

                        Float distance = results[0] ;

                        double lat1 = lastLocation.getLatitude() ;
                        double lng1 = lastLocation.getLongitude() ;

                        lat1 = Math.toRadians(lat1 );
                        lng1 = Math.toRadians(lng1) ;

                        float radBearing = (float) (bearing * 180 / Math.PI) ;

                        double angularDistance = distance / radius ;


                        double deltaLat = angularDistance * Math.cos(radBearing);
                        double lat2 = lat1 + deltaLat;

                        double deltaPsi = Math.log(Math.tan(lat2 / 2 + Math.PI / 4) / Math.tan(lat1 / 2 + Math.PI / 4));
                        double q;
                        if(Math.abs(deltaPsi)>10e-12){
                            q = deltaLat/deltaPsi ;

                        }else{
                            q = Math.cos(deltaLat) ;
                        }

                        //double q = Math.abs(deltaPsi) > 10e-12 ? deltaLat / deltaPsi : Math.cos(deltaLat);

                        double deltaLng = angularDistance * Math.sin(radBearing) / q;

                        double lng2 = lng1 + deltaLng;

                        lat2 = Math.toDegrees(lat2);
                        lng2 = Math.toDegrees(lng2);

                        if (!(-180<lng2 && lng2<=180)) {

                            lng2 = (lng2+540)%360-180;

                        };

                        //set new location point

                        Log.i("MyActivity", "set new location point " );

                        mapCircle.remove();

                        mMap.setMinZoomPreference(18);

                        currentLocation = new LatLng(lat2,lng2);

                        circleOptions.center(currentLocation);

                        circleOptions.radius(6);
                        circleOptions.fillColor(Color.GREEN);
                        circleOptions.strokeWidth(6);

                        mapCircle = mMap.addCircle(circleOptions);
                        mMap.moveCamera(CameraUpdateFactory.newLatLng(currentLocation));

                        accelCount = 0;
                    }

                }

            }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public class AverageDistance {

        int index = 0;
        List<Float> xs = new ArrayList<Float>() ;
        List<Float> ys = new ArrayList<Float>() ;
        List<Float> zs = new ArrayList<Float>() ;

        List<Long> times = new ArrayList<Long>() ;
        Long GPSTimestamp = 0L ;

        public AverageDistance(){

            xs.add(0.0f) ;
            ys.add(0.0f) ;
            zs.add(0.0f) ;

            times.add(0L) ;

        }
        void updateGPSTimestamp(Long timestamp){

            GPSTimestamp = timestamp;

        }

        Float[] getAverageDistance(){

            Float distanceX = 0.0f ;
            Float distanceY = 0.0f ;
            Float distanceZ = 0.0f ;

            Long deltaT = 0L;

            if(index >= 7){

                //gets the average of the last 7 values
                for(int j=index; j > index - 7; j -- ){

                    deltaT = times.get(j) - GPSTimestamp/1000;

                    Float velocity = 0.0f;

                    //integrate twice for x
                    velocity = xs.get(j) * deltaT ;
                    Float dist = velocity * deltaT ;
                    distanceX += dist;

                    //integerate twice for y
                    velocity = ys.get(j) * deltaT;
                    dist = velocity * deltaT ;
                    distanceY += dist ;

                    //integrate twice for z
                    velocity = zs.get(j) * deltaT ;
                    dist = velocity * deltaT ;
                    distanceZ += dist;
                }

            }else if (index >= 1){

                //for when there are less than 5 values, uses however many values are available

                for(int j=index; j > 0 ; j -- ){

                    deltaT = times.get(j) - GPSTimestamp/1000;

                    Float velocity = 0.0f;

                    //integrate twice for x
                    velocity = xs.get(j) * deltaT ;
                    Float dist = velocity * deltaT ;
                    distanceX += dist;

                    //integerate twice for y
                    velocity = ys.get(j) * deltaT;
                    dist = velocity * deltaT ;
                    distanceY += dist ;

                    //integrate twice for z
                    velocity = zs.get(j) * deltaT ;
                    dist = velocity * deltaT ;
                    distanceZ += dist;

                }

            }

            Float[] distance = new Float[3];
            distance[0] = distanceX;
            distance[1] = distanceY ;
            distance[2] = distanceZ ;

            return distance;
        }
    }

    @Override
    public void onLocationChanged(Location location) {

        isUpdating = true;

        tsLong = System.currentTimeMillis();

        myChunk.updateGPSTimestamp(tsLong);

        mapCircle.remove();

        mMap.setMinZoomPreference(18);

        currentLocation = new LatLng(location.getLatitude(),location.getLongitude());

        bearing = location.getBearing() ;

        lastLocation = location ;

        circleOptions.center(currentLocation);

        circleOptions.radius(6);
        circleOptions.fillColor(Color.RED);
        circleOptions.strokeWidth(6);

        mapCircle = mMap.addCircle(circleOptions);
        mMap.moveCamera(CameraUpdateFactory.newLatLng(currentLocation));

        isUpdating = false;

    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {

    }

    @Override
    public void onProviderEnabled(String provider) {

    }

    @Override
    public void onProviderDisabled(String provider) {

    }

    @Override
    protected void onStop() {
        super.onStop();

        mySensorManager.unregisterListener(this);

    }

    @Override
    protected void onPause() {
        super.onPause();
        mySensorManager.unregisterListener(this);

    }
}
