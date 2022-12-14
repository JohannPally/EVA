package com.example.eva3;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import androidx.annotation.NonNull;
import android.content.pm.PackageManager;
import android.Manifest;
import android.app.Fragment;
import android.content.Context;
import android.graphics.Bitmap;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraManager;
import android.media.Image;
import android.media.ImageReader;
import android.media.MediaPlayer;
import android.os.Build;
import android.os.Bundle;
import android.os.StrictMode;
import android.util.Log;
import android.util.Size;
import android.view.Surface;
import android.view.View;
import android.widget.Button;

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.ArrayDeque;
import java.util.Queue;

/**
 * Main activity class for implementing image transmission
 */
public class MainActivity extends AppCompatActivity implements ImageReader.OnImageAvailableListener {

    private Socket socket;

    /**
     * Initialize main components of the application, create listeners and button mappings
     * @param savedInstanceState
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Button initSocket = findViewById(R.id.button_start_socket);
        View.OnClickListener startSocketListener = (v) -> {
            try {
                initSocket();
            } catch (IOException e) {
                e.printStackTrace();
            }
        };
        initSocket.setOnClickListener(startSocketListener);

        Button closeSocket = findViewById(R.id.button_close_socket);
        View.OnClickListener closeSocketListener = (v) -> {
            try {
                closeSocket();
            } catch (IOException e) {
                e.printStackTrace();
            }
        };
        closeSocket.setOnClickListener(closeSocketListener); // set button listenrs and mapping

        int SDK_INT = android.os.Build.VERSION.SDK_INT;
        if (SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions(this, new String[] {
                    Manifest.permission.CAMERA // permission requests
            }, 121);
        } else {
            setFragment();
        }
        if (SDK_INT > 8) {
            StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder()
                    .permitAll().build();
            StrictMode.setThreadPolicy(policy);
            // allow socket usage in main thread
        }
    }

    boolean isTransmitting = false;
    boolean started = false; // transmission task statuses
    Bitmap rgbFrameBitmap; // global image bitmap object
    int prev_img_size = 0; // guarantees image uniqueness
    long prev_audio = 0;  // ensures audio is not played too frequently

    /**
     * Initialize socket connection, and start a image sending task if it is not already running
     */
    public void initSocket() throws IOException{
        System.out.println("init socket!!!");
        socket = new Socket();
        socket.connect(new InetSocketAddress("192.168.1.76", 6650), 3000);
        // The address and the port number of the cloud server is the only constant needed to run the app
        isTransmitting = true;
        if(!started) {
            System.out.println("Start task!!!");
            new Thread(receivePromptTask).start();
        }
        started = true;
        System.out.println("finished init socket!!!");
    }

    // Image transmission task object started by initSocket()
    Runnable receivePromptTask =
            () -> {
                MediaPlayer mp1 = MediaPlayer.create(this, R.raw.error_1);
                MediaPlayer mp2 = MediaPlayer.create(this, R.raw.error_2);
                MediaPlayer mp3 = MediaPlayer.create(this, R.raw.error_3);
                MediaPlayer mp4 = MediaPlayer.create(this, R.raw.error_4);
                MediaPlayer mp5 = MediaPlayer.create(this, R.raw.error_5);
                MediaPlayer mp6 = MediaPlayer.create(this, R.raw.error_6);
                mp1.setVolume(20, 20);
                mp2.setVolume(20, 20);
                mp3.setVolume(20, 20);
                mp4.setVolume(20, 20);
                mp5.setVolume(20, 20);
                mp6.setVolume(20, 20); // initialize audio players on android
                while(isTransmitting) {
                    try {
                        if(socket != null && socket.isConnected()) {
                            DataInputStream dataInputStream = new DataInputStream(socket.getInputStream());
                            int result = 0;
                            try {
                                if(dataInputStream.available() != 0) {
                                    result = dataInputStream.readByte(); // receive status codes from cloud server
                                }
                            } catch(Exception e) {
                                e.printStackTrace();
                            }

                            if(System.currentTimeMillis() - prev_audio > 2000){ // prevents the audio from playing too frequently
                                if (result == 49) {
                                    mp1.start();
                                } else if (result == 50) {
                                    mp2.start();
                                } else if (result == 51) {
                                    mp3.start();
                                } else if (result == 52) {
                                    mp4.start();
                                } else if (result == 53) {
                                    mp5.start();
                                }  else if (result == 54) {
                                    mp6.start();
                                }
                                prev_audio = System.currentTimeMillis();
                            }
                            ByteArrayOutputStream bos = new ByteArrayOutputStream();
                            rgbFrameBitmap.compress(Bitmap.CompressFormat.JPEG, 50, bos); // compress the current image

                            byte[] img_bytes = bos.toByteArray();
                            if(img_bytes.length == prev_img_size) {
                                continue;
                            } else {
                                prev_img_size = img_bytes.length;
                            }
                            byte[] size_bytes = ByteBuffer.allocate(4).putInt(img_bytes.length).array(); // convert images to java bytes array

                            byte[] buffer = new byte[size_bytes.length + img_bytes.length];
                            System.arraycopy(size_bytes, 0, buffer, 0, size_bytes.length);
                            System.arraycopy(img_bytes, 0, buffer, size_bytes.length, img_bytes.length); // copy both the image size and content bytes in a single array
                            DataOutputStream dataOutputStream = new DataOutputStream(socket.getOutputStream());
                            dataOutputStream.write(buffer, 0, buffer.length); // buffer is sent only once to reduce latency
                            dataOutputStream.flush();
                        }
                    } catch (IOException ignored) {}
                }
            };

    /**
     * Closes the socket connection and set the transmission flag to false, which at the next iteration
     * of the task, will cause the task to be stopped
     */
    public void closeSocket() throws IOException {
        System.out.println("close socket!!!");
        isTransmitting = false;
        started = false;
        socket.close();
    }

    /**
     * Enable fragment on permission request result
     * @param requestCode
     * @param permissions
     * @param grantResults
     */
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            setFragment();
        } else {
            finish();
        }
    }

    int previewHeight = 0, previewWidth = 0; // dimensions of the camera view in app
    /**
     * Creates the camera fragment for the app
     */
    protected void setFragment() {
        final CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        String cameraId = null;
        try {
            cameraId = manager.getCameraIdList()[0];
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
        Fragment fragment;
        CameraConnectionFragment camera2Fragment =
                CameraConnectionFragment.newInstance(
                        (size, rotation) -> {
                            previewHeight = size.getHeight();
                            previewWidth = size.getWidth();
                            Log.d("tryOrientation", "rotation: " + rotation + "   orientation: " + getScreenOrientation() + "  " + previewWidth + "   " + previewHeight);
                        },
                        this,
                        R.layout.camera_fragment,
                        new Size(640, 480));

        camera2Fragment.setCamera(cameraId);
        fragment = camera2Fragment;
        getFragmentManager().beginTransaction().replace(R.id.container, fragment).commit();
    }

    private boolean isProcessingFrame = false;
    private final byte[][] yuvBytes = new byte[3][];
    private int[] rgbBytes = null;
    private int yRowStride;
    private Runnable postInferenceCallback;
    private Runnable imageConverter;

    /**
     * Image pre-processing when an image becomes available
     * Converts from YUV420 to ARGB8888 color space
     * @param reader
     */
    @Override
    public void onImageAvailable(ImageReader reader) {
        // We need wait until we have some size from onPreviewSizeChosen
        if (previewWidth == 0 || previewHeight == 0) {
            return;
        }
        if (rgbBytes == null) {
            rgbBytes = new int[previewWidth * previewHeight];
        }
        try {
            final Image image = reader.acquireLatestImage();

            if (image == null) {
                return;
            }
            if (isProcessingFrame) {
                image.close();
                return;
            }
            isProcessingFrame = true;
            final Image.Plane[] planes = image.getPlanes();
            fillBytes(planes, yuvBytes);
            yRowStride = planes[0].getRowStride();
            final int uvRowStride = planes[1].getRowStride();
            final int uvPixelStride = planes[1].getPixelStride();

            imageConverter =
                    () -> ImageUtils.convertYUV420ToARGB8888(
                            yuvBytes[0],
                            yuvBytes[1],
                            yuvBytes[2],
                            previewWidth,
                            previewHeight,
                            yRowStride,
                            uvRowStride,
                            uvPixelStride,
                            rgbBytes);

            postInferenceCallback =
                    () -> {
                        image.close();
                        isProcessingFrame = false;
                    };

            processImage();

        } catch (final Exception ignored) {}
    }

    /**
     * Creates bitmap by setting pixels from bytes
     */
    private void processImage() {
        imageConverter.run();
        rgbFrameBitmap = Bitmap.createBitmap(previewWidth, previewHeight, Bitmap.Config.ARGB_8888);
        rgbFrameBitmap.setPixels(rgbBytes, 0, previewWidth, 0, 0, previewWidth, previewHeight);
        postInferenceCallback.run();
    }

    /**
     * Helper function to fill buffer with bytes
     * @param planes
     * @param yuvBytes
     */
    protected void fillBytes(final Image.Plane[] planes, final byte[][] yuvBytes) {
        for (int i = 0; i < planes.length; ++i) {
            final ByteBuffer buffer = planes[i].getBuffer();
            if (yuvBytes[i] == null) {
                yuvBytes[i] = new byte[buffer.capacity()];
            }
            buffer.get(yuvBytes[i]);
        }
    }

    /**
     * Retrieve the current camera orientation
     * @return
     */
    protected int getScreenOrientation() {
        switch (getWindowManager().getDefaultDisplay().getRotation()) {
            case Surface.ROTATION_270:
                return 270;
            case Surface.ROTATION_180:
                return 180;
            case Surface.ROTATION_90:
                return 90;
            default:
                return 0;
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }
}