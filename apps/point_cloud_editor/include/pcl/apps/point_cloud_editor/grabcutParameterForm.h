//
//  grabcutParameterForm.h
//  pcl
//
//  Created by Matthew Marzin on 11/18/14.
//  Copyright (c) 2014 Matthew Marzin. All rights reserved.
//

/// @file grabcutParameterForm.h
/// @details
/// @author Matthew Marzin
#ifndef GRABCUT_PARAMETER_FORM_H_
#define GRABCUT_PARAMETER_FORM_H_

#include <QLineEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QLineEdit>

class GrabcutParameterForm : public QDialog
{
    Q_OBJECT
    
public:
    /// @brief Default Constructor
    GrabcutParameterForm();
    
    /// @brief Destructor
    ~GrabcutParameterForm ();
    
    /// @brief Returns the mean
    inline
    float
    getK () const
    {
        return (k_);
    }
    
    /// @brief Returns the standard deviation multiplier threshold
    inline
    float
    getLambda () const
    {
        return (lambda_);
    }
    
    /// @brief Checks whether the OK button was pressed.
    inline
    bool
    ok () const
    {
        return (ok_);
    }
    
    private slots:
    /// @brief Accepts and stores the current user inputs, and turns off the
    /// dialog box.
    void
    accept ();
    
    /// @brief Rejects the current inputs, and turns off the dialog box.
    void
    reject ();
    
private:
    /// The line for entering the mean
    QLineEdit *k_line_;
    /// The line for entering the standard deviation multiplier threshold
    QLineEdit *lambda_line_;
    /// The button box.
    QDialogButtonBox *button_box_;
    /// The layout of the two input QLineEdit objects
    QFormLayout *layout_;
    /// The main layout for the dialog
    QVBoxLayout* main_layout_;
    /// The mean
    float k_;
    /// The standard deviation multiplier threshold
    float lambda_;
    /// The flag indicating whether the OK button was pressed
    bool ok_;
};

#endif // GRABCUT_PARAMETER_FORM_H_

