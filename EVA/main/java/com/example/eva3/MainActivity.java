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

import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.ByteBuffer;

public class MainActivity extends AppCompatActivity implements ImageReader.OnImageAvailableListener {

    private Socket socket;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Button initSocket = findViewById(R.id.button_start_socket);
        View.OnClickListener startSocketListener = (v) -> {
            initSocket();
        };
        initSocket.setOnClickListener(startSocketListener);

        Button closeSocket = findViewById(R.id.button_close_socket);
        View.OnClickListener closeSocketListener = (v) -> {
            closeSocket();
        };
        closeSocket.setOnClickListener(closeSocketListener);

        int SDK_INT = android.os.Build.VERSION.SDK_INT;
        if (SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions(this, new String[] {
                    Manifest.permission.CAMERA
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
    public void initSocket() {
        System.out.println("init socket!!!");
        Runnable processingTask =
            () -> {
                try {
                    isTransmitting = true;
                    socket = new Socket();
                    socket.connect(new InetSocketAddress("192.168.1.76", 6650), 3000);
                    socket.setSendBufferSize(2000);
                    while(isTransmitting) {
                        if(isBitMapReady) {
                            try {
                                ByteArrayOutputStream bos = new ByteArrayOutputStream();
                                rgbFrameBitmap.compress(Bitmap.CompressFormat.JPEG, 50, bos);
                                byte[] array = bos.toByteArray();
                                DataOutputStream dataOutputStream = new DataOutputStream(socket.getOutputStream());
                                //System.out.println("Image Length: " + array.length);
                                dataOutputStream.writeInt(array.length);
                                dataOutputStream.write(array, 0, array.length);
                                //System.out.println("!!!!!");
                                imgCount += 1;
                                //System.out.println("Image Count: " + imgCount);
                                //dataOutputStream.flush();
                                //System.out.println("###" + (System.nanoTime() - prev));
                                isBitMapReady = false;
                                //System.out.println("Image Count: 2");
                                //dataOutputStream.close();
                                //System.out.println("Image Count: 3");
                            } catch (Exception e) {
                                System.out.println("EXCEPTION!!!!!");
                                e.printStackTrace();
                                initSocket();
                            }
                        }
                    }
                    System.out.println("reached!!!");
                    DataOutputStream dataOutputStream = new DataOutputStream(socket.getOutputStream());
                    dataOutputStream.writeInt(30);
                    socket.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            };

        Runnable receivePromptTask =
                () -> {
                    MediaPlayer mp1 = MediaPlayer.create(this, R.raw.left_up);
                    MediaPlayer mp2 = MediaPlayer.create(this, R.raw.right_up);
                    MediaPlayer mp3 = MediaPlayer.create(this, R.raw.foward);
                    MediaPlayer mp4 = MediaPlayer.create(this, R.raw.backward);
                    mp1.setVolume(20, 20);
                    mp2.setVolume(20, 20);
                    mp3.setVolume(20, 20);
                    mp4.setVolume(20, 20);

                    while(isTransmitting) {
                        if(socket != null && socket.isConnected()) {
                            System.out.println(")))))");
                            try {
                                DataInputStream dataInputStream = new DataInputStream(socket.getInputStream());
                                System.out.println("????");
                                int result = Integer.parseInt(dataInputStream.readUTF());
                                System.out.println(result + "!!!!");

                                if (result == 1) {
                                    mp1.start();
                                } else if (result == 2) {
                                    mp2.start();
                                } else if (result == 3) {
                                    mp3.start();
                                } else {
                                    mp4.start();
                                }

                            } catch (IOException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                };
        new Thread(processingTask).start();
        new Thread(receivePromptTask).start();
    }

    public void closeSocket() {
        System.out.println("close socket!!!");
        isTransmitting = false;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            setFragment();
        } else {
            finish();
        }
    }

    int previewHeight = 0, previewWidth = 0;
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
    private int imgCount = 0;
    Bitmap rgbFrameBitmap;
    boolean isBitMapReady = false;
    long prev = System.nanoTime();
    private void processImage() {
        isBitMapReady = false;
        imageConverter.run();
        rgbFrameBitmap = Bitmap.createBitmap(previewWidth, previewHeight, Bitmap.Config.ARGB_8888);
        rgbFrameBitmap.setPixels(rgbBytes, 0, previewWidth, 0, 0, previewWidth, previewHeight);
        isBitMapReady = true;
        //ImageUtils.saveBitmap(rgbFrameBitmap, System.currentTimeMillis() + ".png");
        postInferenceCallback.run();
    }

    protected void fillBytes(final Image.Plane[] planes, final byte[][] yuvBytes) {
        // Because of the variable row stride it's not possible to know in
        // advance the actual necessary dimensions of the yuv planes.
        for (int i = 0; i < planes.length; ++i) {
            final ByteBuffer buffer = planes[i].getBuffer();
            if (yuvBytes[i] == null) {
                yuvBytes[i] = new byte[buffer.capacity()];
            }
            buffer.get(yuvBytes[i]);
        }
    }

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